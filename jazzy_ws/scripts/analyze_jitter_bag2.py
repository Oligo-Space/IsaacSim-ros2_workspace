#!/usr/bin/env python3
"""Follow-up: joint positions vs limits around the t=19s JOINT_BOUND flap; motion timeline."""
import numpy as np
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

BAG = "/home/pj/IsaacSim-ros2_workspace/jazzy_ws/jitter_onset"
LIMITS = {  # rad, from es165_macro.xacro
    "joint_1_s": (np.radians(-180), np.radians(180)),
    "joint_2_l": (np.radians(-130), np.radians(80)),
    "joint_3_u": (np.radians(-112), np.radians(208)),
    "joint_4_r": (np.radians(-360), np.radians(360)),
    "joint_5_b": (np.radians(-130), np.radians(130)),
    "joint_6_t": (np.radians(-360), np.radians(360)),
}

reader = SequentialReader()
reader.open(StorageOptions(uri=BAG, storage_id="mcap"), ConverterOptions("", ""))
type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}

states = []  # measured
while reader.has_next():
    topic, raw, t_ns = reader.read_next()
    if topic != "/isaac_joint_states":
        continue
    msg = deserialize_message(raw, get_message(type_map[topic]))
    states.append((t_ns / 1e9, list(msg.name), np.array(msg.position)))

t0 = states[0][0]
names = states[0][1]
T = np.array([s[0] - t0 for s in states])
P = np.array([s[2] for s in states])

# Only the 6 arm joints, in case RWs are included
arm = [i for i, n in enumerate(names) if n in LIMITS]
print("Joints in /isaac_joint_states:", names)

print("\n=== Measured positions (deg) at key times ===")
for tq in [0.5, 9.7, 12, 15, 18, 19.0, 19.5, 20, 22, 25, 30, 32, 35, 40, 45, 48]:
    i = np.searchsorted(T, tq)
    if i >= len(T):
        break
    row = ", ".join(f"{names[j].split('_')[1]}={np.degrees(P[i, j]):8.2f}" for j in arm)
    print(f"  t={T[i]:6.2f}s  {row}")

print("\n=== Distance to nearest position limit (deg) over time, per joint ===")
for j in arm:
    n = names[j]
    lo, hi = LIMITS[n]
    dist = np.minimum(P[:, j] - lo, hi - P[:, j])
    imin = np.argmin(dist)
    print(f"  {n}: min distance {np.degrees(dist[imin]):7.3f} deg at t={T[imin]:.2f}s "
          f"(pos={np.degrees(P[imin, j]):.2f} deg, limits [{np.degrees(lo):.0f},{np.degrees(hi):.0f}])")

# Velocity profile of the most active joint around 19s
print("\n=== Per-joint total motion 9.7s-19s vs 19s-30s (deg) ===")
i1, i2, i3 = np.searchsorted(T, 9.7), np.searchsorted(T, 19.0), np.searchsorted(T, 30.0)
for j in arm:
    d1 = np.degrees(P[i2, j] - P[i1, j])
    d2 = np.degrees(P[min(i3, len(T)-1), j] - P[i2, j])
    print(f"  {names[j]}: onset->19s {d1:+8.3f}   19s->30s {d2:+8.3f}")
