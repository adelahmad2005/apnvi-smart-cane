# APNVI operator console

A screen on the campus PC that gets the device ready from a cold lab, maps a room and proves the
device finds itself on it, shows what the device is doing, and fixes the known problems with one
press. The device's features stay on voice; this is the operator's panel.

Tested on the PC with a pretend device (below) and the ROS stand-in, then used on the real device for the final demo.

## The two halves

| Where | File | What it does |
|---|---|---|
| Campus PC | `pc/hub.py` (+ `pc/static/index.html`) | The page on http://localhost:8100. Starts and checks the PC's servers (Ollama 11434, speech 8090, voice 8091) from the Startup shortcuts, warms up Qwen, keeps the ssh tunnel (8081 device page, 8082 helper), runs "Get ready", passes the page's device buttons to the helper |
| Jetson | `jetson/helper.py` (+ `links.py`) | Always on from boot, 127.0.0.1:8082 only. Starts and stops the device with `device/start.sh` and `stop.sh`, one ROS node that listens to the device and sends the few commands the console may send |

What the console may send to the device: the map commands (new, save, load, entrance, rename,
remove), one buzz or one grip motor for a few seconds, STOP (cancel the trip and one zero speed),
emptying the obstacle maps, and start/stop/restart/reset of the device. **It never drives the
wheels.** Hard stop ends the drive link program, so the master stops the wheels by itself.

## Set up (once)

Jetson (after `git pull`):
```
sudo cp ~/apnvi_demo/src/apnvi-smart-cane/console/jetson/apnvi-console.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now apnvi-console.service
```
Don't also install `device/apnvi.service` (it would start the device by itself at boot).
Without the service, the PC's "Start the Jetson's helper" button starts the helper over ssh.

PC: a desktop shortcut to `pc/start_console.bat`. Optional settings in `pc/hub_config.json`
(the keys of `DEFAULTS` in `hub.py`: ssh host, ports, the Startup shortcuts, the Qwen model).

## Use

1. Double-click the shortcut. The page opens.
2. **Get ready**: pick Guided demo or Mapping demo (labeller on), press Get ready. Every line goes
   green, yellow (works, but look) or red (with a Fix button).
3. **Map a room**: New map, walk, Mark entrance, Save as `esb117_live`, move, Load. "Where am I"
   shows searching, then placed, and the device says "Found where you are."
4. **Something went wrong**: red boxes appear by themselves with a Fix button; the table below
   lists the known problems and their buttons.
5. **STOP** (or the space bar) cancels the trip. Green on the grip stays the real stop.

## Try it with no device

Double-click `pc/try_fake.bat` (or run `python jetson/helper.py --fake` and `python pc/hub.py --fake`).
The page shows a FAKE device label. Pretend problems for practice:
`curl -X POST localhost:8082/api/fake/trouble -d "{\"grip\": false}"` (also `"camera": false`,
`"placement": "lost"`, `"guide": "guiding", "route": false`).

## Tests

`python console/tests/test_console.py` (8 tests, about 30 s): the
start options, the map guard, the problem finder, the helper's API, the whole Get ready run and the
ROS link through `sim/rosstub`.
