# Whole-chain tests with no hardware (16 Nov)

Two tests that run the real programs of Stages 2 to 4 together on any computer, with no ROS, no
camera, no LiDAR, no AI and no device. They were written to check the final build before it went on
the Jetson, and they are the "whole-chain test" in `docs/results.md`.

| File | What it runs | What it shows |
|---|---|---|
| `chain_test.py` | Map manager, Stage 2, the labeller, the guide, the vibration program, the health program | 11 steps: "Ready", the map labelling itself, "take me to the door", arrival, a thing not in view, a thing found and pinned, "continue", "save this place as", "scan the room", "forget the door" |
| `free_mode_test.py` | Free mode steering, the hazard gate, the vibration program | Nobody pushes: no turning. Pushed at a box: turns away, forward speed 0, the "going right" signal. Let go: stops. While guiding: sends nothing |
| `rosstub/` | A stand-in for ROS 2 and its launch system | Each message goes straight to whoever listens. Enough to import and drive the programs, and to run the packages' tests on a computer without ROS |

Needs Python with numpy and OpenCV. Run from the repository's top folder:

```
P=sim/rosstub; for i in 1 2 3 4; do P=$P:src/apnvi_stage$i; done
PYTHONPATH=$P python3 sim/chain_test.py
PYTHONPATH=$P python3 sim/free_mode_test.py
```

What these tests can't show: that the camera, the LiDAR, the motors, the grip, the network and the
real AI behave. That is what the lab tests on the device are for (see `docs/results.md`).
