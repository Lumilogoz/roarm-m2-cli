#!/usr/bin/env python3
"""Minimal CLI for Waveshare RoArm-M2-S over USB serial.

Coordinate frame (firmware RoArm-M2_config.h, verified against live feedback):
  x, y  horizontal distance in mm from the base rotation axis; +x is straight
        out from the arm, +y is to its left. The base joint rotates to face
        (x, y) automatically, so x/y are absolute, not joint-relative.
  z     vertical distance in mm from the SHOULDER PIVOT, which sits ~126 mm
        above the desk (ARM_L1_LENGTH_MM = 126.06). So z = 0 is shoulder
        height, not the table; the desk is roughly z = -126.
  t     end-effector (gripper/wrist) angle in RADIANS, not degrees.

Physical envelope (firmware link lengths + joint constraints):
  reach      sqrt(236.82^2 + 30^2) + sqrt(280.15^2 + 1.73^2) = 518.9 mm
  base       +/-180 deg
  shoulder   +/-90 deg
  elbow      -45 .. +180 deg
  hand       700..3396 servo steps -> t 1.07 .. 5.21 rad (1.57 release, 3.14 grab)

Vendor limits (Waveshare wiki / repo README): 4 DOF, 850 g body, payload
0.5 kg at 0.5 m, 1 m working diameter, 0.088 deg repeatability, 12-bit encoders,
7-12.6 V input (12 V 5 A supply). The power switch (item 12 on the board)
controls servo power; USB alone gives serial replies but no motion.

    ./roarm.py limits
    ./roarm.py status
    ./roarm.py move 300 0 200 1.57
    ./roarm.py move 300 0 200 1.57 --speed 0.15 --wait
"""

import argparse
import glob
import json
import math
import sys
import time

BAUD = 115200
L1_MM = 126.06          # base to shoulder pivot
L2_MM = math.hypot(236.82, 30.00)
L3_MM = math.hypot(280.15, 1.73)
REACH_MAX_MM = L2_MM + L3_MM            # 518.9
REACH_MIN_MM = abs(L3_MM - L2_MM)       # 41.5, arm fully folded
Z_MIN_MM, Z_MAX_MM = -310.0, 515.0      # joint-limited vertical envelope
DESK_Z_MM = -L1_MM                      # z of a desk the base stands on
T_MIN_RAD, T_MAX_RAD = 1.07, 5.21       # hand servo 700..3396 steps
SPD_MIN, SPD_MAX = 0.05, 1.0



def discover_port():
    """Return the single USB-serial port; stale suffixes are not assumed."""
    found = []
    for pattern in ("/dev/cu.usbserial-*", "/dev/cu.usbmodem-*", "/dev/ttyUSB*", "/dev/ttyACM*"):
        found.extend(sorted(glob.glob(pattern)))
    if len(found) == 1:
        return found[0]
    if not found:
        raise SystemExit("no USB serial port found — check the CP210x cable")
    raise SystemExit("multiple serial ports, pass --port:\n  " + "\n  ".join(found))


def validate(x, y, z, t, spd):
    """Reject targets outside the arm's physical workspace before touching hardware."""
    for name, value in (("x", x), ("y", y), ("z", z), ("t", t), ("speed", spd)):
        if not math.isfinite(value):
            raise SystemExit(f"{name} must be a finite number")
    radius = math.hypot(x, y)
    if radius > REACH_MAX_MM:
        raise SystemExit(f"unreachable: horizontal radius {radius:.1f} mm > {REACH_MAX_MM:.1f} mm")
    if radius < REACH_MIN_MM:
        raise SystemExit(f"too close: horizontal radius {radius:.1f} mm < {REACH_MIN_MM:.1f} mm (arm folds into itself)")
    if not Z_MIN_MM <= z <= Z_MAX_MM:
        raise SystemExit(f"z out of range: {z} mm not in [{Z_MIN_MM:.0f}, {Z_MAX_MM:.0f}]")
    if z < DESK_Z_MM:
        print(f"warning: z={z} mm is below the shoulder pivot by {-z:.0f} mm; "
              f"anything under z={DESK_Z_MM:.0f} mm is below desk level", file=sys.stderr)
    if not T_MIN_RAD <= t <= T_MAX_RAD:
        raise SystemExit(f"t out of range: {t} not in [{T_MIN_RAD:.2f}, {T_MAX_RAD:.2f}]")
    if not SPD_MIN <= spd <= SPD_MAX:
        raise SystemExit(f"speed out of range: {spd} not in [{SPD_MIN}, {SPD_MAX}]")


def send(port, payload, expect=None, timeout=4.0):
    """Write one newline-terminated JSON command; return the reply with matching 'T'."""
    import serial  # imported late so --selftest works without pyserial

    with serial.Serial(port, BAUD, timeout=0.4, dsrdtr=None) as link:
        link.setRTS(False)
        link.setDTR(False)
        time.sleep(0.25)
        link.reset_input_buffer()
        link.write(json.dumps(payload, separators=(",", ":")).encode() + b"\n")
        link.flush()
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            line = link.readline()
            if not line:
                continue
            try:
                reply = json.loads(line.decode(errors="replace").strip())
            except json.JSONDecodeError:
                continue
            if expect is None or reply.get("T") == expect:
                return reply
    raise SystemExit(f"no reply to T={payload['T']} within {timeout}s — is the arm powered?")


def cmd_status(port):
    reply = send(port, {"T": 105}, expect=1051)
    print(json.dumps(reply, indent=2))
    print(
        f"\nbase {math.degrees(reply['b']):.1f}°  shoulder {math.degrees(reply['s']):.1f}°  "
        f"elbow {math.degrees(reply['e']):.1f}°  hand {math.degrees(reply['t']):.1f}°\n"
        f"radius {math.hypot(reply['x'], reply['y']):.1f} mm of {REACH_MAX_MM:.1f} mm  |  "
        f"z {reply['z']:.1f} mm = {reply['z'] - DESK_Z_MM:.1f} mm above desk  |  "
        f"t {reply['t']:.2f} rad",
        file=sys.stderr,
    )
    return reply


def cmd_limits():
    deg = math.degrees
    print(f"""RoArm-M2-S envelope           value          meaning
  x, y                        mm             horizontal from base axis (+x forward, +y left)
  z                           mm             vertical from shoulder pivot; desk at z={DESK_Z_MM:.0f}
  t                           rad            end-effector angle (1.57 release, 3.14 grab)
  max reach                   {REACH_MAX_MM:.1f} mm     sqrt(236.82^2+30^2)+sqrt(280.15^2+1.73^2)
  min radius                  {REACH_MIN_MM:.1f} mm      arm fully folded
  z range                     {Z_MIN_MM:.0f} .. {Z_MAX_MM:.0f} mm
  t range                     {T_MIN_RAD:.2f} .. {T_MAX_RAD:.2f} rad  ({deg(T_MIN_RAD):.0f}..{deg(T_MAX_RAD):.0f} deg, hand servo 700..3396)
  base joint                  +/-180 deg
  shoulder joint              +/-90 deg
  elbow joint                 -45 .. +180 deg
  move speed                  {SPD_MIN} .. {SPD_MAX} (fraction of travel per interpolated step)

Vendor data                  value
  4 DOF, body <850 g, payload 0.5 kg at 0.5 m, working diameter ~1 m
  repeatability 0.088 deg, 12-bit magnetic encoders, ST3215 bus servos @1 Mbaud
  power 7-12.6 V, use 12 V 5 A; board power switch must be ON for servos to move
  serial 115200 baud, newline-terminated JSON, T:104 = XYZT goal, T:1041 = direct, T:105 = read""")


def cmd_move(args, port):
    validate(args.x, args.y, args.z, args.t, args.speed)
    payload = {"T": 104, "x": args.x, "y": args.y, "z": args.z, "t": args.t, "spd": args.speed}
    print(f"-> {json.dumps(payload)}", file=sys.stderr)
    start = cmd_status(port)
    send(port, payload, expect=None, timeout=2.0)
    if not args.wait:
        return
    deadline = time.monotonic() + args.timeout
    last = start
    while time.monotonic() < deadline:
        time.sleep(1.0)
        last = send(port, {"T": 105}, expect=1051)
        err = math.dist((last["x"], last["y"], last["z"]), (args.x, args.y, args.z))
        if err < 5.0:
            print(f"reached target, residual {err:.1f}mm")
            return
    print(
        f"stopped short: target ({args.x}, {args.y}, {args.z}) "
        f"reached ({last['x']:.1f}, {last['y']:.1f}, {last['z']:.1f})",
        file=sys.stderr,
    )
    sys.exit(3)


def cmd_selftest():
    validate(300, 0, 235, 1.57, 0.25)
    validate(430, -116, -120, 3.21, 0.25)   # live pose read from the arm
    for bad in (
        (999, 0, 100, 1.57, 0.25),   # beyond reach
        (10, 0, 100, 1.57, 0.25),    # inside folded radius
        (300, 0, 900, 1.57, 0.25),   # z too high
        (300, 0, 100, 0.5, 0.25),    # t below hand limit
        (300, 0, 100, 1.57, 2.0),    # speed too high
    ):
        try:
            validate(*bad)
        except SystemExit:
            continue
        raise AssertionError(f"accepted invalid target {bad}")
    print("selftest ok")


def main():
    parser = argparse.ArgumentParser(description="RoArm-M2 XYZT CLI")
    parser.add_argument("--port", help="serial port (default: auto-detect)")
    sub = parser.add_subparsers(dest="cmd", required=True)
    sub.add_parser("limits", help="print coordinate frame, envelope and unit reference")
    sub.add_parser("status", help="read current x/y/z/t (moves nothing)")
    move = sub.add_parser("move", help="move end effector to x y z t")
    move.add_argument("x", type=float, help="mm")
    move.add_argument("y", type=float, help="mm")
    move.add_argument("z", type=float, help="mm")
    move.add_argument("t", type=float, help="end-effector angle, radians")
    move.add_argument("--speed", type=float, default=0.25, help="0.05-1.0, default 0.25")
    move.add_argument("--wait", action="store_true", help="poll until the target is reached")
    move.add_argument("--timeout", type=float, default=20.0, help="wait budget in seconds")
    sub.add_parser("selftest", help="validate bounds logic without hardware")

    args = parser.parse_args()
    if args.cmd == "selftest":
        return cmd_selftest()
    if args.cmd == "limits":
        return cmd_limits()
    port = args.port or discover_port()
    if args.cmd == "status":
        cmd_status(port)
    elif args.cmd == "move":
        cmd_move(args, port)


if __name__ == "__main__":
    main()
