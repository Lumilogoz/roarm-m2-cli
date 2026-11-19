# roarm-m2-cli

Minimal XYZT command-line controller for the Waveshare RoArm-M2-S robotic arm over USB serial.

## Usage

| Command | Effect |
|---|---|
| `./roarm.py limits` | Print coordinate frame, envelope and unit reference |
| `./roarm.py status` | Read current x/y/z/t (moves nothing) |
| `./roarm.py move X Y Z T` | Move end effector; add `--speed 0.15 --wait` to poll arrival |
| `./roarm.py selftest` | Check bounds logic without hardware |

```bash
python3 -m pip install pyserial
./roarm.py status
./roarm.py move 300 0 200 1.57 --wait
```

## Coordinate frame

| Axis | Unit | Meaning |
|---|---|---|
| x, y | mm | Horizontal from the base rotation axis; +x forward, +y left. The base turns to face (x, y). |
| z | mm | Vertical from the shoulder pivot, not the desk. The desk is about z = -126. |
| t | rad | End-effector angle; 1.57 releases, 3.14 grips. |

## Limits

| Limit | Value |
|---|---|
| Max reach | 518.9 mm |
| Min radius | 41.4 mm |
| z range | -310 to 515 mm |
| t range | 1.07 to 5.21 rad |
| Base / shoulder / elbow | ±180° / ±90° / -45° to 180° |
| Body, payload, repeatability | 850 g, 0.5 kg at 0.5 m, 0.088° |
| Power | 7–12.6 V, 12 V 5 A; the board power switch must be on for servos to move |

Values come from the [Waveshare RoArm-M2 firmware](https://github.com/waveshareteam/roarm_m2) link lengths and joint constraints, cross-checked against the vendor wiki and the ROS 2 URDF.
