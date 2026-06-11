#!/usr/bin/env python3
"""Post-process a fresh Isaac URDF import of arm_w_mm.usd.

Re-importing the URDF resets drive gains to importer defaults (~1-7k stiffness),
which is ~1000x too soft to track 200 Hz position commands -> the arm sags and
jiggles. This restores the hand-tuned gains and unlocks the continuous wrist.
Run after every re-import, then reload the stage in Isaac.

Requires: pip install usd-core
"""
import math
import sys

from pxr import Usd

USD = "/home/pj/IsaacSim-ros2_workspace/jazzy_ws/src/es165_moveit/urdf/arm_w_mm/arm_w_mm.usd"
ARM_JOINTS = ["joint_1_s", "joint_2_l", "joint_3_u", "joint_4_r", "joint_5_b", "joint_6_t"]
STIFFNESS = 1.0e7  # matches the hand-tuned value the original stage used

stage = Usd.Stage.Open(sys.argv[1] if len(sys.argv) > 1 else USD)
fixed = []
for prim in stage.Traverse():
    name = prim.GetName()
    if name in ARM_JOINTS and str(prim.GetTypeName()) == "PhysicsRevoluteJoint":
        prim.GetAttribute("drive:angular:physics:stiffness").Set(STIFFNESS)
        fixed.append(name)
stage.GetRootLayer().Save()

assert sorted(fixed) == sorted(ARM_JOINTS), f"only fixed {fixed}"
print(f"set stiffness={STIFFNESS:.0e} on {len(fixed)} joints (limits left as imported)")
print("now reload the stage in Isaac Sim")
