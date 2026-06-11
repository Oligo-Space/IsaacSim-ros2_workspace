#!/usr/bin/env python3
"""Compare commanded EE angular velocity (servo twist cmds) vs achieved (FK on measured
joint states) from a rosbag. Usage: python3 verify_ee_rates.py [bag_dir]"""
import sys
import numpy as np
import pinocchio as pin
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

BAG = sys.argv[1] if len(sys.argv) > 1 else "/home/pj/IsaacSim-ros2_workspace/jazzy_ws/jitter_onset"
URDF = "/tmp/es165.urdf"

model = pin.buildModelFromUrdf(URDF)
data = model.createData()
ee_id = model.getFrameId("ee_base_link")
joint_order = [model.names[i] for i in range(1, model.njoints)]

reader = SequentialReader()
reader.open(StorageOptions(uri=BAG, storage_id="mcap"), ConverterOptions("", ""))
type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}

cmds, states, torques = [], [], []
while reader.has_next():
    topic, raw, t_ns = reader.read_next()
    t = t_ns / 1e9
    if topic == "/servo_node/delta_twist_cmds":
        m = deserialize_message(raw, get_message(type_map[topic]))
        cmds.append((t, np.array([m.twist.angular.x, m.twist.angular.y, m.twist.angular.z])))
    elif topic == "/isaac_joint_states":
        m = deserialize_message(raw, get_message(type_map[topic]))
        q = np.zeros(model.nq)
        ok = True
        for jn, qi in zip(joint_order, range(model.nq)):
            if jn in m.name:
                q[qi] = m.position[m.name.index(jn)]
            else:
                ok = False
        if ok:
            states.append((t, q))
    elif topic == "/torque_input":
        m = deserialize_message(raw, get_message(type_map[topic]))
        torques.append((t, list(m.data)))

t0 = states[0][0]
print(f"{len(cmds)} twist cmds, {len(states)} joint states, torques at "
      f"{[round(t - t0, 2) for t, _ in torques]}")

# FK: EE rotation matrix per sample -> body angular velocity via finite difference
T = np.array([s[0] - t0 for s in states])
R = []
for _, q in states:
    pin.framesForwardKinematics(model, data, q)
    R.append(data.oMf[ee_id].rotation.copy())

omega_body = []  # body-frame angular velocity between consecutive samples
for i in range(1, len(R)):
    dt = T[i] - T[i - 1]
    if dt <= 0:
        omega_body.append(np.zeros(3))
        continue
    dR = R[i - 1].T @ R[i]            # body-frame incremental rotation
    w = pin.log3(dR) / dt             # axis-angle rate, body frame
    omega_body.append(w)
omega_body = np.array(omega_body)
Tm = T[1:]

cmd_t = np.array([c[0] - t0 for c in cmds])
cmd_w = np.array([c[1] for c in cmds])

def window_stats(t_lo, t_hi, label):
    mi = (Tm >= t_lo) & (Tm <= t_hi)
    ci = (cmd_t >= t_lo) & (cmd_t <= t_hi)
    if not mi.any() or not ci.any():
        return
    ach = omega_body[mi].mean(axis=0)
    cmd = cmd_w[ci].mean(axis=0)
    print(f"\n[{label}]  t={t_lo}..{t_hi}s")
    print(f"  commanded twist (ee frame): {np.round(cmd, 4)}  |w|={np.linalg.norm(cmd):.4f} rad/s")
    print(f"  achieved EE body rate (FK): {np.round(ach, 4)}  |w|={np.linalg.norm(ach):.4f} rad/s")
    n = np.linalg.norm
    if n(cmd) > 1e-6 and n(ach) > 1e-6:
        cos = np.dot(cmd, ach) / (n(cmd) * n(ach))
        print(f"  magnitude ratio achieved/commanded: {n(ach)/n(cmd):.3f}   "
              f"direction alignment cos: {cos:.4f}")

# windows: before first torque, after each torque (coast), tail
events = [t - t0 for t, _ in torques]
if events:
    window_stats(max(0, events[0] - 3), events[0] - 0.1, "pre-torque")
    window_stats(events[0] + 1.0, events[0] + 6.0, "coast after torque 1")
    if len(events) > 1:
        window_stats(events[-1] + 1.0, events[-1] + 6.0, "coast after last torque")

# overall norm trace, decimated
print("\n|w| commanded vs achieved over time (1s bins):")
for tb in np.arange(0, Tm[-1], 1.0):
    mi = (Tm >= tb) & (Tm < tb + 1)
    ci = (cmd_t >= tb) & (cmd_t < tb + 1)
    if mi.any() and ci.any():
        print(f"  t={tb:5.1f}s  cmd={np.linalg.norm(cmd_w[ci].mean(axis=0)):6.3f}  "
              f"ach={np.linalg.norm(omega_body[mi].mean(axis=0)):6.3f}")
