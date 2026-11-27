// MASTER ESP32 (Stage 4, 12 Nov): drives 2x BTS7960 (IBT_2) motor drivers, talks to the grip
// ESP32 over ESP-NOW (direct radio link), and stops the wheels BY ITSELF when it is not safe.
//
// From the Jetson over USB (115200 baud, one command per line):
//   d <left> <right>   wheel power, each -100..100 (percent of the power cap, + = forward).
//                      The Jetson sends this about 10 times a second, even when it is 0 0.
//   vibe <0-100> [1|2|3] grip vibration strength, kept on while repeated; off 0.5 s after the
//                      last one, and "vibe 0" switches it off at once. The second number
//                      picks the grip motor (16 Nov); left out = motor 1;
//                      3 = both motors (17 Nov: "near" and "stopped" on both)
//   rear on|off        switch the "nobody behind the device" stop on or off (default: REAR_CHECK)
//   who                answers "who master" (so the Jetson knows it opened the right board)
//   Old test commands, kept: f b l r s, lf lb, fw, vib <1|2|0> <ms>, power 0-100, slave,
//   ping, raw on|off, tx <hex>, p (pin test, ONLY with the 12 V unplugged).
//   The wheel test commands obey the same stop rules, so f drives for 0.5 s unless repeated.
//
// The master stops both wheels at once, by itself, when any of these is true:
//   red      the grip's red button is pressed
//   green    the grip's green button (emergency stop) is on
//   handle   no radio message from the grip for 0.3 s
//   jetson   no wheel command from the Jetson for 0.5 s
//   rear     nobody within about 1 m behind the device (rear HC-SR04; only when switched on)
// After a stop, the wheels stay stopped until every reason is gone AND the Jetson has sent a
// zero command (d 0 0 or s). Then the master says "ready". So nothing lurches back to life.
//
// To the Jetson: "stop <reason>" when a stop reason starts, "clear <reason>" when it ends,
// "ready", "btn red down/up", "btn green on/off", "slave connected/lost", "who master",
// and once a second: "alive <s> slave <up|down> stops <list|none> l <pwm> r <pwm> rear <m>".
//
// RED_DRIVE_TEST (test build only, OFF by default): hold the grip's red button -> both wheels
// forward at TEST_POWER; let go (0.3 s filter) -> stop. In this build red is NOT a stop reason,
// the Jetson-quiet rule is off and "d" commands are ignored, so nothing has to run on the Jetson.
// Green, handle-quiet and rear still stop the wheels; after a stop, let go of red to drive again.
// Build it with:  arduino-cli compile --build-property "compiler.cpp.extra_flags=-DRED_DRIVE_TEST=1" ...
#ifndef RED_DRIVE_TEST
#define RED_DRIVE_TEST 0
#endif
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

const int LED_PIN = 2;

// Left driver
const int L_RPWM = 25, L_LPWM = 26, L_REN = 27, L_LEN = 4;
// Right driver
const int R_RPWM = 32, R_LPWM = 33, R_REN = 14, R_LEN = 13;

const int PWM_FREQ = 20000;
const int PWM_BITS = 8;
const int MAX_PWM = 100;        // POWER CAP: 100% from the Jetson = this PWM (out of 255).
                                // 100 is the speed tested on 11 Nov. Measure the real speed here.
const int TEST_POWER = 100;     // percent used by the old test commands f b l r lf lb fw
const int DEADTIME_MS = 200;    // a wheel rests at 0 this long before changing direction
const float RAMP_PCT_PER_S = 200.0;   // speeding up: 0 -> 100% in 0.5 s. Stopping is instant.

// ---- Stop rules: settings ----
const unsigned long HANDLE_QUIET_MS = 300;   // grip radio quiet this long -> stop
const unsigned long JETSON_QUIET_MS = 500;   // no wheel command this long -> stop
const unsigned long RED_RELEASE_MS = 300;    // red counts as let go only after 0.3 s off
                                             // (its contact flickers)
const unsigned long VIBE_HOLD_MS = 500;      // "vibe N" lasts this long unless repeated

// Rear ultrasonic (HC-SR04 facing the user). NOT WIRED YET: off by default.
// ECHO gives 5 V: it MUST go through a voltage divider (for example 1k + 2k) to 3.3 V.
const bool REAR_CHECK = false;               // true once the sensor is wired and tested
const int REAR_TRIG = 18;                    // GPIO 18 -> HC-SR04 TRIG
const int REAR_ECHO = 19;                    // GPIO 19 <- HC-SR04 ECHO (through the divider)
const float REAR_MAX_M = 1.0;                // the user must be closer than this
const unsigned long REAR_EVERY_MS = 60;      // one reading every 60 ms
const int REAR_MISSES = 3;                   // this many far readings in a row -> stop

enum Reason { R_RED = 0, R_GREEN, R_HANDLE, R_JETSON, R_REAR, N_REASONS };
const char *REASON_NAMES[N_REASONS] = {"red", "green", "handle", "jetson", "rear"};
bool reasonOn[N_REASONS] = {false, false, true, true, false};   // at boot: nothing heard yet
bool needZero = true;            // after a stop: wait for a zero command before driving
bool zeroSeen = false;           // a zero command came in since the last stop

struct Motor { int rpwm, lpwm, ren, len; };
Motor leftM  = {L_RPWM, L_LPWM, L_REN, L_LEN};
Motor rightM = {R_RPWM, R_LPWM, R_REN, R_LEN};

// One wheel: what the Jetson wants (target) and what it gets now (now), in percent.
struct Wheel { const Motor *m; float target; float now; unsigned long restUntil; };
Wheel leftW  = {&leftM, 0, 0, 0};
Wheel rightW = {&rightM, 0, 0, 0};
unsigned long lastDriveCmd = 0;
unsigned long fwRightAt = 0;     // "fw": when the right wheel joins (0 = not waiting)

// ---- ESP-NOW link to slave (broadcast, no MAC pairing needed) ----
const uint8_t BCAST[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
const uint8_t MAGIC = 0xA7;
enum MsgType : uint8_t { M_HEART = 1, M_VIB = 2, S_PONG = 10, S_BTN = 11 };
struct __attribute__((packed)) Msg { uint8_t magic, type, a; uint16_t b; };
QueueHandle_t rxQ, rawQ;
const uint8_t CHILD[6] = {0xFC,0xE8,0xC0,0xE0,0xE4,0x84};
bool rawLog = false;
bool cmdM1 = false, cmdM2 = false; uint8_t cmdPower = 100; uint16_t gripSeq = 0;
unsigned long m1OffAt = 0, m2OffAt = 0;
uint8_t vibeStrength = 0; unsigned long vibeOffAt = 0;   // "vibe N" from the Jetson
uint8_t vibeMotor = 1;                                   // "vibe N 2" = the grip's second motor
uint8_t gRed = 255, gGreen = 255, gM1 = 255, gM2 = 255, gP = 255;
struct Raw { uint8_t mac[6]; uint8_t len; uint8_t d[48]; };
unsigned long lastSlaveMsg = 0, lastHeart = 0;
bool slaveUp = false;
bool redHeld = false;            // red after the release filter
unsigned long redOffSince = 0;   // when the raw red signal last went off (0 = it is on)

// Rear sensor state
bool rearCheck = REAR_CHECK;
float rearM = -1;                // last reading in metres, -1 = no echo
int rearFar = 0;                 // far readings in a row
unsigned long lastRear = 0;

void setReason(int r, bool on) {
  if (reasonOn[r] == on) return;
  reasonOn[r] = on;
  Serial.printf("%s %s\n", on ? "stop" : "clear", REASON_NAMES[r]);
}

bool anyReason() {
  for (int i = 0; i < N_REASONS; i++) if (reasonOn[i]) return true;
  return false;
}

void onRecv(const esp_now_recv_info_t *info, const uint8_t *d, int n) {
  Raw r; memcpy(r.mac, info->src_addr, 6); r.len = n > 48 ? 48 : n; memcpy(r.d, d, r.len);
  xQueueSendFromISR(rawQ, &r, NULL);
  if (n != sizeof(Msg)) return;
  Msg m; memcpy(&m, d, sizeof m);
  if (m.magic == MAGIC) xQueueSendFromISR(rxQ, &m, NULL);
}
void sendSlave(uint8_t type, uint8_t a, uint16_t b) {
  Msg m = {MAGIC, type, a, b};
  esp_now_send(BCAST, (const uint8_t *)&m, sizeof m);
}
void setupEspNow() {
  rxQ = xQueueCreate(16, sizeof(Msg));
  rawQ = xQueueCreate(16, sizeof(Raw));
  WiFi.mode(WIFI_STA); WiFi.disconnect();
  if (esp_now_init() != ESP_OK) { Serial.println("espnow init FAILED"); return; }
  esp_now_peer_info_t peer = {}; memcpy(peer.peer_addr, BCAST, 6);
  peer.channel = 0; peer.encrypt = false;
  esp_now_add_peer(&peer);
  esp_now_peer_info_t cp = {}; memcpy(cp.peer_addr, CHILD, 6); cp.channel = 0; cp.encrypt = false;
  esp_now_add_peer(&cp);
  esp_now_register_recv_cb(onRecv);
  Serial.printf("espnow ready, master MAC %s\n", WiFi.macAddress().c_str());
}

// Red: pressed counts at once (stop), let go counts only after RED_RELEASE_MS off.
void updateRed(uint8_t raw, unsigned long now) {
  if (raw) {
    redOffSince = 0;
    if (!redHeld) { redHeld = true; Serial.println("btn red down"); }
  } else if (redHeld) {
    if (redOffSince == 0) redOffSince = now ? now : 1;
    else if (now - redOffSince >= RED_RELEASE_MS) { redHeld = false; Serial.println("btn red up"); }
  }
  setReason(R_RED, RED_DRIVE_TEST ? false : redHeld);   // test build: red drives, never stops
}

void pollSlave(unsigned long now) {
  if (cmdM1 && (long)(now - m1OffAt) >= 0) cmdM1 = false;
  if (cmdM2 && (long)(now - m2OffAt) >= 0) cmdM2 = false;
  if (vibeStrength && (long)(now - vibeOffAt) >= 0) vibeStrength = 0;
  if (now - lastHeart >= 50) {             // GRIP parent packet: "GRIP",1,seq16,m1,m2,power
    lastHeart = now; gripSeq++;
    bool vibeOn = vibeStrength > 0;
    bool m1 = cmdM1 || (vibeOn && vibeMotor != 2);       // 1 or 3
    bool m2 = cmdM2 || (vibeOn && vibeMotor >= 2);       // 2 or 3
    uint8_t power = vibeOn ? vibeStrength : ((cmdM1 || cmdM2) ? cmdPower : 0);
    uint8_t pk[10] = {0x50,0x49,0x52,0x47, 1, (uint8_t)(gripSeq & 0xFF), (uint8_t)(gripSeq >> 8),
                      (uint8_t)m1, (uint8_t)m2, power};
    // power 0 unless the Jetson asks for vibration -> the grip's own button vibration is silent
    esp_now_send(CHILD, pk, sizeof pk);
  }
  Raw r;
  bool gotRed = false; uint8_t rawRed = 0;
  while (xQueueReceive(rawQ, &r, 0) == pdTRUE) {
    if (r.len == 12 && r.d[0]==0x50 && r.d[1]==0x49 && r.d[2]==0x52 && r.d[3]==0x47 && r.d[4]==2) {
      lastSlaveMsg = now;
      if (!slaveUp) { slaveUp = true; Serial.println("slave connected"); }
      gotRed = true; rawRed = r.d[7];
      if (r.d[8] != gGreen && (gGreen != 255 || r.d[8])) Serial.printf("btn green %s\n", r.d[8] ? "on" : "off");
      if (r.d[7]!=gRed || r.d[8]!=gGreen || r.d[9]!=gM1 || r.d[10]!=gM2 || r.d[11]!=gP) {
        gRed=r.d[7]; gGreen=r.d[8]; gM1=r.d[9]; gM2=r.d[10]; gP=r.d[11];
        if (rawLog) Serial.printf("grip red=%u green=%u m1=%u m2=%u p=%u\n", gRed, gGreen, gM1, gM2, gP);
      }
    }
    if (!rawLog) continue;
    Serial.printf("RAW from %02X:%02X:%02X:%02X:%02X:%02X len %u:", r.mac[0],r.mac[1],r.mac[2],r.mac[3],r.mac[4],r.mac[5], r.len);
    for (int i = 0; i < r.len; i++) Serial.printf(" %02X", r.d[i]);
    Serial.println();
  }
  if (gotRed) updateRed(rawRed, now);
  else if (redHeld && redOffSince) updateRed(0, now);   // no packet now: keep the filter running
  Msg m;
  while (xQueueReceive(rxQ, &m, 0) == pdTRUE) {
    lastSlaveMsg = now;
    if (!slaveUp) { slaveUp = true; Serial.println("slave connected"); }
    if (m.type == S_BTN) Serial.printf("btn %u %s\n", m.a, m.b ? "down" : "up");
  }
  bool handleQuiet = (lastSlaveMsg == 0) || (now - lastSlaveMsg > HANDLE_QUIET_MS);
  setReason(R_HANDLE, handleQuiet);
  setReason(R_GREEN, gGreen == 1 && !handleQuiet);
  if (handleQuiet) {
    // Nothing known about the buttons while the grip is quiet: start fresh when it is back.
    // (The "handle" reason already stops the wheels.)
    gRed = gGreen = 255;
    if (redHeld) { redHeld = false; redOffSince = 0; Serial.println("btn red up"); }
    setReason(R_RED, false);
  }
  if (slaveUp && now - lastSlaveMsg > 1500) { slaveUp = false; Serial.println("slave lost"); }
}

// ---- Rear HC-SR04: is somebody behind the device? ----
float readRearM() {
  digitalWrite(REAR_TRIG, LOW); delayMicroseconds(2);
  digitalWrite(REAR_TRIG, HIGH); delayMicroseconds(10);
  digitalWrite(REAR_TRIG, LOW);
  unsigned long us = pulseIn(REAR_ECHO, HIGH, 12000);   // 12 ms = about 2 m, then give up
  if (us == 0) return -1;
  return us * 0.000343 / 2.0;
}

void pollRear(unsigned long now) {
  if (!rearCheck) { rearFar = 0; setReason(R_REAR, false); return; }
  if (now - lastRear < REAR_EVERY_MS) return;
  lastRear = now;
  rearM = readRearM();
  bool far = rearM < 0 || rearM > REAR_MAX_M;
  rearFar = far ? rearFar + 1 : 0;
  if (rearFar >= REAR_MISSES) setReason(R_REAR, true);
  else if (!far) setReason(R_REAR, false);
}

// ---- Motors ----
void motorStop(const Motor &m) {
  ledcWrite(m.rpwm, 0);
  ledcWrite(m.lpwm, 0);
  digitalWrite(m.ren, LOW);
  digitalWrite(m.len, LOW);
}

// pct: -100..100 of MAX_PWM. + = forward (RPWM on both wheels, tested 11 Nov).
void motorOut(const Motor &m, float pct) {
  int pwm = (int)(fabs(pct) * MAX_PWM / 100.0 + 0.5);
  if (pwm <= 0) { motorStop(m); return; }
  digitalWrite(m.ren, HIGH);
  digitalWrite(m.len, HIGH);
  if (pct > 0) { ledcWrite(m.lpwm, 0); ledcWrite(m.rpwm, pwm); }
  else         { ledcWrite(m.rpwm, 0); ledcWrite(m.lpwm, pwm); }
}

void stopAll() {
  motorStop(leftM);
  motorStop(rightM);
  leftW.now = rightW.now = 0;
  leftW.target = rightW.target = 0;
  fwRightAt = 0;
}

// Move one wheel toward its target: speeding up is gradual, slowing and stopping are instant,
// and a change of direction rests at 0 for DEADTIME_MS first.
void stepWheel(Wheel &w, float dt, unsigned long now) {
  float t = w.target;
  if (w.now != 0 && t != 0 && (t > 0) != (w.now > 0)) {   // change of direction
    w.now = 0; motorOut(*w.m, 0); w.restUntil = now + DEADTIME_MS; return;
  }
  if ((long)(w.restUntil - now) > 0) { motorOut(*w.m, 0); return; }
  if (fabs(t) <= fabs(w.now)) {                              // slower: at once
    if (t == 0 && w.now != 0) w.restUntil = now + DEADTIME_MS;
    w.now = t;
  }
  else {
    float step = RAMP_PCT_PER_S * dt;
    if (t > w.now) w.now = min(t, w.now + step); else w.now = max(t, w.now - step);
  }
  motorOut(*w.m, w.now);
}

unsigned long lastStep = 0;
void driveLoop(unsigned long now) {
  setReason(R_JETSON, !RED_DRIVE_TEST && (lastDriveCmd == 0 || now - lastDriveCmd > JETSON_QUIET_MS));
#if RED_DRIVE_TEST
  // Red held = forward, red let go = the zero command. Overrides anything the Jetson sent.
  leftW.target = rightW.target = redHeld ? TEST_POWER : 0;
  if (!redHeld) zeroSeen = true;
  fwRightAt = 0;
#endif
  if (anyReason()) {
    stopAll();
    needZero = true;
    zeroSeen = false;
    lastStep = now;
    return;
  }
  if (needZero) {                       // reasons gone: wait for a fresh zero command
    if (zeroSeen && leftW.target == 0 && rightW.target == 0) {
      needZero = false; Serial.println("ready");
    } else { motorStop(leftM); motorStop(rightM); lastStep = now; return; }
  }
  if (fwRightAt && (long)(now - fwRightAt) >= 0) { rightW.target = TEST_POWER; fwRightAt = 0; }
  float dt = (now - lastStep) / 1000.0;
  lastStep = now;
  stepWheel(leftW, dt, now);
  stepWheel(rightW, dt, now);
}

// A wheel command from the Jetson (d, or one of the test commands).
void setTargets(float l, float r) {
  leftW.target = constrain(l, -100, 100);
  rightW.target = constrain(r, -100, 100);
  if (leftW.target == 0 && rightW.target == 0) zeroSeen = true;
  lastDriveCmd = millis();
  if (lastDriveCmd == 0) lastDriveCmd = 1;
}

void setupMotor(const Motor &m) {
  pinMode(m.ren, OUTPUT); digitalWrite(m.ren, LOW);
  pinMode(m.len, OUTPUT); digitalWrite(m.len, LOW);
  ledcAttach(m.rpwm, PWM_FREQ, PWM_BITS); ledcWrite(m.rpwm, 0);
  ledcAttach(m.lpwm, PWM_FREQ, PWM_BITS); ledcWrite(m.lpwm, 0);
}

// ---- Pin connectivity test: digital inputs only, nothing is driven, motors cannot move ----
static int readDig(int pin, int mode) {
  pinMode(pin, mode);          // INPUT_PULLUP or INPUT_PULLDOWN
  delay(30);
  int hi = 0;
  for (int i = 0; i < 20; i++) { hi += digitalRead(pin); delay(1); }
  return hi;                   // 0..20 samples HIGH
}

void pinTest() {
  stopAll();
  struct P { int pin; const char *name; bool motor; };
  const P pins[] = {
    {12, "REF unused GPIO12", false}, {15, "REF unused GPIO15", false},
    {25, "L_RPWM", true}, {26, "L_LPWM", true}, {27, "L_R_EN", true}, {4, "L_L_EN", true},
    {32, "R_RPWM", true}, {33, "R_LPWM", true}, {14, "R_R_EN", true}, {13, "R_L_EN", true},
  };
  Serial.println("PINTEST start (digital inputs only, nothing driven)");
  Serial.println("pin  name               pullup_hi/20  pulldown_hi/20  verdict");
  for (auto &p : pins) {
    if (p.motor) ledcDetach(p.pin);
    int up = readDig(p.pin, INPUT_PULLUP);
    int dn = readDig(p.pin, INPUT_PULLDOWN);
    const char *v;
    if (up >= 18 && dn <= 2)       v = "FLOATING (nothing connected / open)";
    else if (up <= 2 && dn <= 2)   v = "HELD LOW (driver pulldown, or short to GND)";
    else if (up >= 18 && dn >= 18) v = "HELD HIGH (short to 3V3?)";
    else                           v = "UNCLEAR";
    Serial.printf("%-4d %-18s %-13d %-15d %s\n", p.pin, p.name, up, dn, v);
    if (p.motor) { pinMode(p.pin, INPUT); }
  }
  setupMotor(leftM);
  setupMotor(rightM);
  needZero = true;
  zeroSeen = false;
  Serial.println("PINTEST done, pins restored, motors stopped");
}

void printStops() {
  bool any = false;
  for (int i = 0; i < N_REASONS; i++) {
    if (!reasonOn[i]) continue;
    Serial.print(any ? "," : ""); Serial.print(REASON_NAMES[i]); any = true;
  }
  if (!any) Serial.print(needZero ? "waitzero" : "none");
}

void handleLine(String s) {
  s.trim();
  s.toLowerCase();
  if (s.length() == 0) return;
  // d <left> <right>: wheel power in percent of the cap, sent ~10 times a second
  if (s.startsWith("d ")) {
    int sp = s.indexOf(' ', 2);
    if (sp < 0) { Serial.print("bad: "); Serial.println(s); return; }
    setTargets(s.substring(2, sp).toFloat(), s.substring(sp + 1).toFloat());
    return;
  }
  if (s.startsWith("vibe ")) {
    int v = constrain(s.substring(5).toInt(), 0, 100);
    int sp = s.indexOf(' ', 5);
    int m = sp > 0 ? s.substring(sp + 1).toInt() : 1;   // 17 Nov: 3 = both grip motors
    vibeMotor = (m == 2 || m == 3) ? m : 1;
    vibeStrength = v; vibeOffAt = millis() + VIBE_HOLD_MS;
    return;
  }
  if (s == "who")  { Serial.println("who master"); return; }
  if (s == "ping") { Serial.println("pong"); return; }
  if (s == "p") { pinTest(); return; }
  if (s == "rear on")  { rearCheck = true;  Serial.println("rear check on");  return; }
  if (s == "rear off") { rearCheck = false; Serial.println("rear check off"); return; }
  // vibration: "vib 1 300" / "vib 2 300" (ms, max 5000), "vib 0" = off, "power 0-100"
  // child runs one motor at a time (motor 1 wins)
  if (s.startsWith("vib")) {
    char w = s.length() > 4 ? s[4] : '0';
    int ms = constrain(s.length() > 6 ? s.substring(6).toInt() : 0, 0, 5000);
    unsigned long t = millis();
    if (w == '1' && ms > 0)      { cmdM1 = true; m1OffAt = t + ms; }
    else if (w == '2' && ms > 0) { cmdM2 = true; m2OffAt = t + ms; }
    else { cmdM1 = cmdM2 = false; vibeStrength = 0; }
    Serial.printf("vib m1=%d m2=%d %d ms%s\n", cmdM1, cmdM2, ms, slaveUp ? "" : " (slave not connected!)");
    return;
  }
  if (s.startsWith("power ")) { cmdPower = constrain(s.substring(6).toInt(), 0, 100); Serial.printf("power %u\n", cmdPower); return; }
  if (s == "raw on")  { rawLog = true;  Serial.println("raw on");  return; }
  if (s == "raw off") { rawLog = false; Serial.println("raw off"); return; }
  if (s.startsWith("tx ")) {             // tx <hex bytes>  -> send raw to child
    uint8_t buf[48]; int n = 0; String h = s.substring(3); h.replace(" ", "");
    for (int i = 0; i + 1 < (int)h.length() && n < 48; i += 2) buf[n++] = strtoul(h.substring(i, i + 2).c_str(), NULL, 16);
    esp_now_send(CHILD, buf, n);
    return;
  }
  if (s == "slave") { Serial.println(slaveUp ? "slave connected" : "slave not connected"); return; }
  // Old wheel test commands: same stop rules as "d", so each lasts 0.5 s unless repeated.
  if (s == "fw") {                       // staggered: left (LPWM) first, right (RPWM) 300 ms later
    setTargets(-TEST_POWER, 0); fwRightAt = millis() + 300;
    Serial.println("fw: left first (LPWM), right joins in 0.3 s");
    return;
  }
  if (s == "lf") { setTargets(TEST_POWER, 0);  Serial.println("left only forward"); return; }
  if (s == "lb") { setTargets(-TEST_POWER, 0); Serial.println("left only back");    return; }
  if (s.length() == 1) {
    switch (s[0]) {
      case 'f': setTargets(TEST_POWER, TEST_POWER);   Serial.println("forward"); return;
      case 'b': setTargets(-TEST_POWER, -TEST_POWER); Serial.println("back");    return;
      case 'l': setTargets(-TEST_POWER, TEST_POWER);  Serial.println("left");    return;
      case 'r': setTargets(TEST_POWER, -TEST_POWER);  Serial.println("right");   return;
      case 's': setTargets(0, 0);                     Serial.println("stopped"); return;
    }
  }
  Serial.print("unknown: "); Serial.println(s);
}

String rx;
unsigned long lastBeat = 0;
bool ledOn = false;

void setup() {
  // Enable pins LOW first, before anything else
  setupMotor(leftM);
  setupMotor(rightM);
  pinMode(LED_PIN, OUTPUT);
  pinMode(REAR_TRIG, OUTPUT); digitalWrite(REAR_TRIG, LOW);
  pinMode(REAR_ECHO, INPUT);

  Serial.begin(115200);
  delay(500);
  Serial.println();
  Serial.println("=== ESP32 boot ===");
  Serial.printf("Chip: %s rev %d\n", ESP.getChipModel(), ESP.getChipRevision());
  Serial.printf("CPU: %u MHz\n", ESP.getCpuFreqMHz());
  Serial.printf("Flash: %u bytes (%u MB)\n", ESP.getFlashChipSize(), ESP.getFlashChipSize() / (1024 * 1024));
  setupEspNow();
  Serial.printf("who master. Power cap PWM %d of 255. Rear check %s.\n", MAX_PWM, rearCheck ? "on" : "off");
  if (RED_DRIVE_TEST) Serial.println("RED DRIVE TEST BUILD: hold red = forward, let go = stop. Jetson commands ignored.");
  Serial.println("Commands: d <l> <r>, vibe <0-100>, rear on|off, who, f b l r s lf lb fw, ping, slave, vib, power. Motors are stopped.");
}

void loop() {
  unsigned long now = millis();
  pollSlave(now);
  pollRear(now);
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') { handleLine(rx); rx = ""; }
    else if (rx.length() < 32) rx += c;
  }
  now = millis();
  driveLoop(now);
  if (now - lastBeat >= 1000) {
    lastBeat = now;
    ledOn = !ledOn;
    digitalWrite(LED_PIN, ledOn);
    Serial.printf("alive %lu s, slave %s stops ", now / 1000, slaveUp ? "up" : "down");
    printStops();
    Serial.printf(" l %d r %d rear %.2f\n", (int)leftW.now, (int)rightW.now, rearM);
  }
  delay(2);
}
