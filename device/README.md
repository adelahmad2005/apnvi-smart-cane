# Session tools: running and testing the whole device

Run on the Jetson, from the campus PC with `ssh jetson`.

```
cd ~/apnvi_demo/src/apnvi-smart-cane/device
source /opt/ros/humble/setup.bash && source ~/apnvi_ws/install/setup.bash
```

## Once: the device's own settings

`start.sh` reads `~/apnvi_device.conf` at every start: one launch option per line.
Copy `apnvi_device.conf.example` to `~/apnvi_device.conf` and fill in the campus PC's address, the
voice, the map, the camera's tilt and the grip motors. Options typed after `start.sh` win over the file.

## Start, stop, look

| Command | What it does |
|---|---|
| `bash start.sh` | Clears leftovers, starts the whole device (Stages 1 to 4) with the settings file, top speed 0.3 m/s, then prints `health`. Finds the Stage 1 ESP32 by itself (or runs Stage 1 on the camera alone) |
| `bash start.sh 0.5` | The same at 0.5 m/s. More options go after the speed: `bash start.sh 0.3 drive:=false labels:=false` |
| `bash stop.sh` | Stops everything (Ctrl+C first, then whatever is left) |
| `python3 session.py health` | Stage 1 rate (want 10), position rate (want 5), the map, the master and grip, what Stage 1 sees, the guide, each part's health, how many pins, Stage 2's state |
| `python3 session.py where` | Where the device is on the map: x, y in metres, heading in degrees |
| `python3 session.py record` | Logs everything once a second and on every change to `~/apnvi_logs/session_<date>_<time>.log`: position, Stage 1, wheels, the guide, what is said, vibration signals, free mode, pins, labels, health. Run it in a second ssh window for the whole session; Ctrl+C ends it |
| `python3 session.py snap` | Saves the camera's picture now to `~/apnvi_logs/snaps/`. `python3 ~/apnvi_demo/src/apnvi-smart-cane/tools/stage2/try_labels.py ~/apnvi_logs/snaps --url http://<campus PC>:11434` then shows what the labeller's question finds in them |
| `python3 floor_check.py` | The camera's height and how far it is tipped down, read from an empty floor. The tilt goes into `camera_tilt:=` |

## The 8081 page: the device's screen, mouth and ears

On the campus PC or a laptop: `ssh -L 8081:localhost:8081 jetson`, then open **http://localhost:8081**.
Click **Sound on** once. The page then plays everything the device says and records the question with
that computer's microphone while the talk button is held (the grip's red button, or the page's
Listening button). Use headphones, so the microphone doesn't record the device's own voice.

Opened as `http://<the Jetson's address>:8081` the page still plays sound but the browser won't give it
the microphone.

The page shows the camera with the answer's red box, Stage 1's level, the map with its pins, and one
line that says what the device is doing.

## Moving the device (someone at the device, green in reach)

| Command | What it does |
|---|---|
| `python3 session.py drive 0.2 5` | Forward at 0.2 m/s for 5 s through the hazard gate, then how far it went and the real speed. **Moves the device** |
| `python3 session.py turn 0.5 3` | Turn on the spot to the left at 0.5 rad/s for 3 s. **Moves the device** |
| `python3 session.py goal "4 0"` | Take me to the map point (4, 0). A name works too: `goal door`. `goal cancel` drops the route. **Moves the device** |
| `python3 session.py say "take me to the door"` | A typed question, as if spoken. Prints what the device says for 25 s. **"Take me to ..." moves the device** |
| `python3 session.py ask continue` | A follow-up, as if the user said "continue" (`ask other`: any other question) |

Drive, turn and goal send speeds the same way the route follower does, so the hazard gate, the grip
buttons, the master's stop rules and the top speed all still apply. Green stops everything.

## The map and its pins

| Command | What it does |
|---|---|
| `python3 session.py map save esb117 final` | Save the running map with its pins. Also `map load esb117 final`, `map new` |
| `python3 session.py pins` | List the pins: name, kind (place, sign, room, thing), map point |
| `python3 session.py place my desk` | Pin the spot where the device stands ("save this place as my desk") |
| `python3 session.py pin remove bin` | Take a pin off the map. `pin rename bin to recycling bin` renames one |
| `python3 session.py scan` | "Scan the room": the labeller looks every 2 s for 60 s while someone turns the device slowly all the way round; then the pins |
| `python3 session.py say "where is the water bottle"` | Stage 2 finds it in the camera picture, places it on the map and pins it |

## Checks

| Command | What it does |
|---|---|
| `python3 session.py ahead 1.5` | Is the spot 1.5 m ahead blocked on the two planners' obstacle maps? Put a box there first: both must say BLOCKED |
| `python3 session.py clear` | Empty both planners' obstacle maps (old camera marks). The walls come back within a second |
| `python3 session.py signal left 3` | Play one vibration signal for 3 s: `careful`, `stopped`, `left`, `right`, `heard`, `arrived`, `help`. `signal motor1 2` and `signal motor2 2` run one grip motor steadily: which side is each on? |
| `python3 session.py reset` | Restart the master ESP32 over USB by hand (the drive link now does this by itself). Run `stop.sh` first |

## Starting at boot (optional)

`apnvi.service` starts the whole device when the Jetson has started. How to install it is at the top
of that file. While it is installed, stop it before running `start.sh` by hand.

## If something is wrong

| What | Look at |
|---|---|
| Nothing starts, or it stops at once | `/tmp/apnvi_session.log` |
| No sound in the browser | "Sound on" clicked? The page opened as `http://localhost:8081`? Is the campus PC's voice server up (`voice_engine:=kokoro`)? Without it the Jetson's own voice (espeak-ng) is used |
| "I can't reach the helper computer" | `pc:=` in `~/apnvi_device.conf`; on the campus PC, Ollama (11434), the speech server (8090) and the voice server (8091) must be running |
| A thing is found but not pinned | The log line says why: `no_depth` (too close, shiny, or tiny), `too_far` (over 5 m), `no_position` (the map hasn't placed the device), `moved` (the device moved while asking) |
| The labeller pins nothing | `ros2 topic echo /stage2/labels`: seconds per look, how many listed and pinned, the problem if any. It waits while Stage 2 is busy and while the device doesn't know where it is |
| The route won't go round or through | `python3 session.py clear`, then try again |
| The master stays silent | `health` shows `restarts`. After 3 tries the drive link gives up: check the master's USB and power, then `stop.sh`, `session.py reset`, `start.sh` |
