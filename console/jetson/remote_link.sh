#!/bin/bash
# Remote link: keep this Jetson reachable over the internet, off the hotspot.
#
# On AUS wireless the PC cannot reach the Jetson directly. This keeps a Cloudflare tunnel to the
# Jetson's own ssh open (it dials out, so no network blocks it) and tells the PC's front door
# where that tunnel is, every 30 s; the PC's `ssh jetson-remote` and the console read it there
# (ai_server/jetson_link.py). The tunnel gets a new address when it starts again: the next report
# carries it.
#
# Settings: ~/apnvi_remote.conf, one line (the PC's fixed front door address, with its token):
#     door=https://<the PC's ngrok address>/k/<token>
# Needs ~/apnvi_tools/cloudflared (ARM build). Started at boot by apnvi-remote-link.service.
# Never touches the device's programs or the network settings.

CONF=~/apnvi_remote.conf
CF=~/apnvi_tools/cloudflared
LOG=/tmp/apnvi_ssh_tunnel.log
door=$(sed -n 's/^door=//p' "$CONF" 2>/dev/null | tr -d '\r')
if [ -z "$door" ]; then
  echo "no door= line in $CONF: nothing to report to" >&2
  exit 1
fi
[ -x "$CF" ] || { echo "no $CF" >&2; exit 1; }

METRICS=127.0.0.1:20241     # cloudflared's own state: /ready is 200 while it is connected
TUNNEL=""
address=""
start_tunnel() {
  "$CF" tunnel --no-autoupdate --protocol http2 --metrics "$METRICS" \
    --url ssh://localhost:22 > "$LOG" 2>&1 &
  TUNNEL=$!
  address=""
  echo "$(date +%T) ssh tunnel starting (pid $TUNNEL)"
}
# Connected to Cloudflare? Asked of cloudflared itself, not by name: a fresh address takes a
# while to be published, and looking it up early fails (18 Nov: that made this throw away
# working tunnels).
ready() { [ "$(curl -s -m 5 -o /dev/null -w '%{http_code}' "http://$METRICS/ready")" = "200" ]; }
# 23 Nov (EB2): after the Jetson moved to another access point cloudflared still said "ready", but
# its public address answered 530 (no tunnel there) and nobody could connect; the PC was told the
# dead address every 30 s. So the address itself is asked too: Cloudflare answers 200 while the
# tunnel is behind it, 530 when it is gone (000: no answer at all).
public_ok() {
  code=$(curl -s -m 10 -o /dev/null -w '%{http_code}' "$address/")
  [ "$code" != "530" ] && [ "$code" != "000" ]
}
trap '[ -n "$TUNNEL" ] && kill "$TUNNEL" 2>/dev/null' EXIT

start_tunnel
reported=""
failed=0
while true; do
  if ! kill -0 "$TUNNEL" 2>/dev/null; then
    echo "$(date +%T) ssh tunnel stopped: starting it again"
    start_tunnel
  fi
  if [ -z "$address" ]; then
    address=$(grep -o 'https://[a-z0-9-]*\.trycloudflare\.com' "$LOG" | head -1)
    if [ -n "$address" ]; then
      for i in $(seq 1 30); do ready && break; sleep 2; done
      sleep 10                  # let Cloudflare publish the new name before the PC looks it up
      echo "$(date +%T) ssh tunnel at $address"
      failed=0
    fi
  fi
  if [ -n "$address" ]; then
    # Still connected? (18 Nov: after a Wi-Fi switch it took 3 min to come back by itself.)
    if ready && public_ok; then
      failed=0
    else
      failed=$((failed + 1))
      echo "$(date +%T) ssh tunnel not connected ($failed)"
      if [ "$failed" -ge 3 ]; then
        echo "$(date +%T) starting a fresh ssh tunnel"
        kill "$TUNNEL" 2>/dev/null
        wait "$TUNNEL" 2>/dev/null
        start_tunnel
        continue
      fi
    fi
    if curl -s -m 15 -o /dev/null -w '%{http_code}' -H 'Content-Type: application/json' \
         -d "{\"ssh\": \"$address\"}" "$door/jetson" | grep -q 200; then
      [ "$reported" != "$address" ] && echo "$(date +%T) told the PC" && reported="$address"
    else
      [ -n "$reported" ] && echo "$(date +%T) the PC's front door does not answer"
      reported=""
    fi
    sleep 20
  else
    sleep 2
  fi
done
