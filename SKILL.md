---
name: roarm-m2-control
description: "Use when controlling a Waveshare RoArm-M2-S arm over serial."
version: 1.0.0
license: MIT
platforms: [macos, linux, windows]
metadata:
  hermes:
    tags: [robotics, serial, roarm, waveshare, esp32]
    category: hardware
---

# RoArm-M2-S control

Operate a Waveshare RoArm-M2-S (ESP32 + ST3215 serial-bus servos) through its
USB serial JSON protocol. Applies to the RoArm-M2-S / M2-Pro family running the
official Waveshare firmware 0.84 or later.

## Coordinate frame

| Axis | Unit | Meaning |
|---|---|---|
| x, y | mm | Horizontal distance from the base rotation axis. +x points forward from the arm, +y to its left. The base rotates to face (x, y), so targets are absolute, not joint-relative. |
| z | mm | Vertical distance from the **shoulder pivot**, not the desk. The pivot sits 126.06 mm above the desk, so desk level is about z = -126. |
| t | rad | End-effector angle. 1.57 releases the gripper, 3.14 grips. |

A reading of z = -120.5 means the tool is 5.5 mm above a desk the base stands
on. Treating z = 0 as the table is the most common frame mistake.

## Envelope and limits

| Limit | Value | Source |
|---|---|---|
| Max reach | 518.9 mm | `hypot(236.82, 30) + hypot(280.15, 1.73)` from firmware link lengths |
| Min radius | 41.4 mm | arm fully folded |
| z range | -310 to 515 mm | joint limits |
| t range | 1.07 to 5.21 rad | hand servo 700..3396 steps |
| Base / shoulder / elbow | ±180° / ±90° / -45..180° | firmware `constrain()` calls |
| Body, payload, repeatability | <850 g, 0.5 kg at 0.5 m, 0.088° | vendor specification |
| Power | 7-12.6 V, 12 V 5 A | vendor specification |

Arrival tolerance observed on hardware is 5-6 mm, so do not claim exact
positioning.

## Transport

- Host channel: USB serial at **115200** baud, newline-terminated JSON. The
  internal servo bus runs at 1 Mbaud over separate pins; do not confuse the two.
- Enumerate `/dev/cu.usbserial-*` (macOS), `/dev/ttyUSB*` or `/dev/ttyACM*`
  (Linux) **at runtime**. Suffixes change between reconnects; never reuse a
  cached path.
- HTTP mode serves `GET /js?json=<url-encoded command>` on port 80. Verify the
  live address from the OLED or `{"T":405}` rather than trusting a sample IP.
- Factory AP is SSID `RoArm-M2` / password `12345678` at 192.168.4.1, but the
  device may have been reconfigured.

## Commands

| Command | Effect |
|---|---|
| `{"T":105}` | Read pose. Reply has `T:1051`, `x/y/z`, `b/s/e/t` (radians), and `torB/torS/torE/torH`. |
| `{"T":104,"x":..,"y":..,"z":..,"t":..,"spd":0.25}` | Interpolated move to XYZT. `spd` 0.05-1.0. |
| `{"T":1041,"x":..,"y":..,"z":..,"t":..}` | Direct move, no interpolation. |
| `{"T":121,"joint":1,"angle":30,"spd":5,"acc":5}` | One joint, **absolute degrees**. Re-sending the same target is not a jog. |
| `{"T":122,"b":0,"s":0,"e":90,"h":180,"spd":10,"acc":10}` | All joints in degrees. |
| `{"T":101}` / `{"T":102}` | Joint control in **radians**. |
| `{"T":106,"cmd":3.14}` | Gripper angle. |
| `{"T":0}` | Disables torque for 10 s, then re-enables. Not a hold-position stop; a loaded arm sags. |

The firmware echoes parsed values (`1  10.00  5  5`) before moving. That echo
proves parsing, not motion — always confirm with `{"T":105}`.

## Procedure

1. Discover the port and confirm the USB-UART bridge is the expected chip.
2. Send `{"T":105}` and parse the pose. If nothing answers, stop and fix the
   transport.
3. Confirm the workspace is clear and the board power switch is on.
4. Send one bounded move at `spd` 0.15 or lower, choosing a target with a
   visibly different coordinate from the measured pose.
5. Query `{"T":105}` again and compare. Report the measured displacement.
6. If it stalls short of target or does not move, stop issuing motion. Inspect
   power supply, physical interference, travel limits, cable routing and
   firmware model match. Never escalate speed or angle to force movement.

## Pitfalls

- **USB connected but silent, all servos reading zero.** The port opens, the
  board enumerates, but `{"T":105}` decodes to the sentinel pose
  `b=π, s=-π, e=-π/2, t=0` with every torque 0. The servo bus is unpowered: the
  board power switch (item 12) is off or the 12 V supply is missing. USB alone
  powers only the ESP32.
- **A small absolute target looks like no motion.** Going from 4.7° to 10°
  moves a few millimetres. Pick a delta that is visible on camera.
- **Stalling part-way with rising load.** Usually mechanical obstruction or a
  weak supply, not a protocol error. A 30° request stopping at 16.5° with load
  climbing is a physical fault.
- **Two different findings conflated.** A process starting, or HTTP 200, or a
  JSON echo, is never evidence of movement.

## Minimal client

The `roarm.py` in this repository wraps the same protocol:

```
./roarm.py limits
./roarm.py status
./roarm.py move 300 0 200 1.57 --speed 0.15 --wait
./roarm.py selftest
```
