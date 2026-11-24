#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Write the bench trainnr fits a servo model against: a one-hinge pendulum model and `bench.json`.

    python3 scripts/servo-bench/make_pendulum.py --length-mm 100 --arm-mass-g 4.0 --weight-mass-g 50.0 \
        --vin 5.8 --kp 15 --joint head_yaw --out <folder>

trainnr's `servo-bench` method (its docs/88-your-own-robot.md) wants the servo's output as the only hinge of a
model, driving an arm whose mass and length are known, with `bench.json` beside it. The arm's inertia and its
gravity torque are what turn a recording of positions into torques, so they have to be measured, not guessed:

  --length-mm      from the servo's axis to the centre of the weight
  --arm-mass-g     the arm alone, with the horn and its screw, weighed
  --weight-mass-g  what is fixed at the arm's end, weighed
  --vin            the supply voltage the servo reports while it is being recorded
  --kp             the servo's position-gain register (address 21) as read from that servo
  --joint          the name the recording gives the servo: the firmware joint `duckd bench --joint` was run with

The arm is taken as a uniform rod from the axis to the weight and the weight as a point at its end. The model's
zero is the arm hanging straight down, with the servo's axis horizontal: switch the servo's torque off, let the arm
come to rest, and that is the joint's zero. A new weight is a new bench: write it to another folder and onboard it
under another name.

Then, with trainnr: `onboard_robot("<folder>/pendulum.xml", "<name>")`, `ingest_recording` for each recording made
with that weight, and `identify_system(robot="<name>", recording="<recording>", method="servo-bench")`.
"""
import argparse
import json
import pathlib

G = 9.81

MODEL = """<mujoco model="servo-bench-pendulum">
  <!-- {about} -->
  <compiler angle="radian"/>
  <option gravity="0 0 -{g}"/>
  <worldbody>
    <body name="arm">
      <joint name="{joint}" type="hinge" axis="0 1 0"/>
      <geom name="rod" type="capsule" fromto="0 0 0 0 0 -{length:.5f}" size="0.002" mass="{arm:.6f}"/>
      <geom name="weight" type="sphere" pos="0 0 -{length:.5f}" size="0.001" mass="{weight:.6f}"/>
    </body>
  </worldbody>
</mujoco>
"""


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--length-mm", type=float, required=True)
    ap.add_argument("--arm-mass-g", type=float, required=True)
    ap.add_argument("--weight-mass-g", type=float, required=True)
    ap.add_argument("--vin", type=float, required=True)
    ap.add_argument("--kp", type=float, required=True)
    ap.add_argument("--joint", required=True)
    ap.add_argument("--servo", default="scs0009", help="the servo family, the fitted model's name")
    ap.add_argument("--out", required=True, help="an empty folder for pendulum.xml and bench.json")
    args = ap.parse_args()
    if min(args.length_mm, args.weight_mass_g, args.vin) <= 0 or args.arm_mass_g < 0:
        ap.error("length, weight and voltage must be more than 0")

    length, arm, weight = args.length_mm / 1000, args.arm_mass_g / 1000, args.weight_mass_g / 1000
    torque = G * length * (weight + arm / 2)  # at 90 degrees, arm horizontal
    inertia = length**2 * (weight + arm / 3)  # a point at the end, a thin rod about its end
    about = (f"arm {args.length_mm:g} mm and {args.arm_mass_g:g} g, weight {args.weight_mass_g:g} g at its end; "
             "zero is the arm hanging down (scripts/servo-bench/make_pendulum.py)")
    out = pathlib.Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    (out / "pendulum.xml").write_text(
        MODEL.format(about=about, g=G, joint=args.joint, length=length, arm=arm, weight=weight))
    bench = {"servo": args.servo, "joint": args.joint, "vin": args.vin, "kp": args.kp, "max_pwm": 1.0}
    (out / "bench.json").write_text(json.dumps(bench) + "\n")
    print(f"wrote {out / 'pendulum.xml'} and {out / 'bench.json'}")
    print(f"  gravity torque with the arm horizontal: {torque:.4f} N*m ({torque / G * 100:.2f} kg*cm)")
    print(f"  inertia about the axis: {inertia:.3e} kg*m^2")


if __name__ == "__main__":
    main()
