#!/usr/bin/env python3
"""Plot jitter_onset bag topics around torque events; saves PNGs to jazzy_ws/."""
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

BAG = "/home/pj/IsaacSim-ros2_workspace/jazzy_ws/jitter_onset"
OUT = "/home/pj/IsaacSim-ros2_workspace/jazzy_ws"

reader = SequentialReader()
reader.open(StorageOptions(uri=BAG, storage_id="mcap"), ConverterOptions("", ""))
type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}

torque = []          # (wall_t, [tx,ty,tz])
twist = []           # (wall_t, header_t, [wx,wy,wz])
ijs = []             # /isaac_joint_states: (wall_t, header_t, pos[6], vel[6])
ijc = []             # /isaac_joint_commands: (wall_t, pos[6], vel[6])
js = []              # /joint_states (broadcaster): (wall_t, header_t, pos[6], vel[6])
ctrl = []            # controller_state: (wall_t, ref_vel[6], act_vel[6], err_pos[6])
clock = []           # (wall_t, sim_t)
status = []          # (wall_t, code)
jt = []              # /arm_controller/joint_trajectory: (wall_t, pos[6], vel[6])

JOINTS = ["joint_1_s", "joint_2_l", "joint_3_u", "joint_4_r", "joint_5_b", "joint_6_t"]

def reorder(names, arr):
    if arr is None or len(arr) == 0:
        return None
    idx = [names.index(j) for j in JOINTS if j in names]
    if len(idx) != 6:
        return None
    return np.asarray(arr)[idx]

while reader.has_next():
    topic, raw, t_ns = reader.read_next()
    if topic not in type_map:
        continue
    t = t_ns / 1e9
    if topic == "/torque_input":
        msg = deserialize_message(raw, get_message(type_map[topic]))
        torque.append((t, list(msg.data)))
    elif topic == "/servo_node/delta_twist_cmds":
        msg = deserialize_message(raw, get_message(type_map[topic]))
        ht = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        tw = msg.twist
        twist.append((t, ht, [tw.angular.x, tw.angular.y, tw.angular.z]))
    elif topic == "/isaac_joint_states":
        msg = deserialize_message(raw, get_message(type_map[topic]))
        ht = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        names = list(msg.name)
        ijs.append((t, ht, reorder(names, msg.position), reorder(names, msg.velocity)))
    elif topic == "/isaac_joint_commands":
        msg = deserialize_message(raw, get_message(type_map[topic]))
        names = list(msg.name)
        ijc.append((t, reorder(names, msg.position), reorder(names, msg.velocity)))
    elif topic == "/joint_states":
        msg = deserialize_message(raw, get_message(type_map[topic]))
        ht = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        names = list(msg.name)
        js.append((t, ht, reorder(names, msg.position), reorder(names, msg.velocity)))
    elif topic == "/arm_controller/controller_state":
        msg = deserialize_message(raw, get_message(type_map[topic]))
        names = list(msg.joint_names)
        rv = reorder(names, msg.reference.velocities) if msg.reference.velocities else None
        av = reorder(names, msg.feedback.velocities) if msg.feedback.velocities else None
        ep = reorder(names, msg.error.positions) if msg.error.positions else None
        ctrl.append((t, rv, av, ep))
    elif topic == "/clock":
        msg = deserialize_message(raw, get_message(type_map[topic]))
        clock.append((t, msg.clock.sec + msg.clock.nanosec / 1e9))
    elif topic == "/servo_node/status":
        msg = deserialize_message(raw, get_message(type_map[topic]))
        status.append((t, msg.code))
    elif topic == "/arm_controller/joint_trajectory":
        msg = deserialize_message(raw, get_message(type_map[topic]))
        if msg.points:
            names = list(msg.joint_names)
            p = msg.points[0]
            jt.append((t, reorder(names, p.positions),
                       reorder(names, p.velocities) if p.velocities else None))

t0 = min(x[0][0] for x in (torque, twist, ijs, clock) if x)
tq_times = [t - t0 for t, _ in torque]
print(f"torque events at: {[f'{x:.2f}' for x in tq_times]}")

def mark_torques(ax):
    for x in tq_times:
        ax.axvline(x, color="red", ls="--", lw=0.8, alpha=0.7)

# ---------- 1. Commanded twist (angular) + inter-message gap ----------
fig, axes = plt.subplots(2, 1, figsize=(14, 8), sharex=True)
tt = np.array([t - t0 for t, _, _ in twist])
ww = np.array([w for _, _, w in twist])
for i, lbl in enumerate("xyz"):
    axes[0].plot(tt, ww[:, i], label=f"w{lbl}", lw=1)
axes[0].set_ylabel("angular vel cmd [rad/s]")
axes[0].set_title("/servo_node/delta_twist_cmds — angular command (red dashed = torque events)")
axes[0].legend(); mark_torques(axes[0]); axes[0].grid(alpha=0.3)
gaps = np.diff(tt) * 1000
axes[1].plot(tt[1:], gaps, lw=0.7)
axes[1].axhline(16.7, color="green", ls=":", label="16.7 ms target")
axes[1].set_ylabel("gap [ms]"); axes[1].set_xlabel("wall time [s]")
axes[1].set_title("twist inter-message gap (wall clock)")
axes[1].legend(); mark_torques(axes[1]); axes[1].grid(alpha=0.3)
fig.tight_layout(); fig.savefig(f"{OUT}/plot1_twist_cmds.png", dpi=120); plt.close(fig)

# ---------- 2. Isaac joint states: velocities ----------
fig, axes = plt.subplots(2, 1, figsize=(14, 8), sharex=True)
ts = np.array([t - t0 for t, _, _, v in ijs if v is not None])
vs = np.array([v for _, _, _, v in ijs if v is not None])
for i, j in enumerate(JOINTS):
    axes[0].plot(ts, vs[:, i], label=j, lw=1)
axes[0].set_ylabel("joint vel [rad/s]")
axes[0].set_title("/isaac_joint_states — joint velocities (physics ground truth)")
axes[0].legend(fontsize=7, ncol=3); mark_torques(axes[0]); axes[0].grid(alpha=0.3)
axes[1].plot(ts, np.linalg.norm(vs, axis=1), lw=1, color="k")
axes[1].set_ylabel("|qdot| [rad/s]"); axes[1].set_xlabel("wall time [s]")
axes[1].set_title("joint velocity norm — drops here = the physical symptom")
mark_torques(axes[1]); axes[1].grid(alpha=0.3)
fig.tight_layout(); fig.savefig(f"{OUT}/plot2_isaac_joint_velocities.png", dpi=120); plt.close(fig)

# ---------- 3. Commanded vs actual joint velocity (per joint, the 3 busiest) ----------
tc = np.array([t - t0 for t, _, v in ijc if v is not None])
vc = np.array([v for t, _, v in ijc if v is not None])
busiest = np.argsort(np.abs(vs).max(axis=0))[::-1][:3] if len(vs) else [0, 1, 2]
fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)
for ax, ji in zip(axes, busiest):
    ax.plot(tc, vc[:, ji], label="cmd vel (/isaac_joint_commands)", lw=0.8, alpha=0.8)
    ax.plot(ts, vs[:, ji], label="actual vel (/isaac_joint_states)", lw=1.2)
    ax.set_ylabel(f"{JOINTS[ji]} [rad/s]"); ax.legend(fontsize=8); ax.grid(alpha=0.3)
    mark_torques(ax)
axes[0].set_title("commanded vs actual joint velocity — 3 most active joints")
axes[-1].set_xlabel("wall time [s]")
fig.tight_layout(); fig.savefig(f"{OUT}/plot3_cmd_vs_actual_vel.png", dpi=120); plt.close(fig)

# ---------- 4. Clock: RTF + sim-time step per /clock msg ----------
fig, axes = plt.subplots(2, 1, figsize=(14, 8), sharex=True)
cw = np.array([t - t0 for t, _ in clock])
cs = np.array([s for _, s in clock])
axes[0].plot(cw[1:], np.diff(cs) * 1000, lw=0.7)
axes[0].set_ylabel("sim dt per /clock msg [ms]")
axes[0].set_title("/clock — sim-time increment per message (16.7 ms = 60 Hz physics)")
mark_torques(axes[0]); axes[0].grid(alpha=0.3)
axes[1].plot(cw[1:], np.diff(cw) * 1000, lw=0.7)
axes[1].set_ylabel("wall gap [ms]"); axes[1].set_xlabel("wall time [s]")
axes[1].set_title("/clock — wall-clock gap between messages (Isaac tick rate)")
mark_torques(axes[1]); axes[1].grid(alpha=0.3)
fig.tight_layout(); fig.savefig(f"{OUT}/plot4_clock.png", dpi=120); plt.close(fig)

# ---------- 5. Controller state: reference vs feedback vel + pos error ----------
if ctrl:
    tcs = np.array([t - t0 for t, rv, av, ep in ctrl if rv is not None and av is not None])
    rvs = np.array([rv for _, rv, av, ep in ctrl if rv is not None and av is not None])
    avs = np.array([av for _, rv, av, ep in ctrl if rv is not None and av is not None])
    eps = np.array([ep for _, rv, av, ep in ctrl if rv is not None and av is not None and ep is not None])
    fig, axes = plt.subplots(2, 1, figsize=(14, 8), sharex=True)
    ji = busiest[0]
    axes[0].plot(tcs, rvs[:, ji], label="reference vel", lw=0.8)
    axes[0].plot(tcs, avs[:, ji], label="feedback vel", lw=1.2, alpha=0.8)
    axes[0].set_ylabel(f"{JOINTS[ji]} [rad/s]")
    axes[0].set_title("/arm_controller/controller_state — reference vs feedback velocity (busiest joint)")
    axes[0].legend(); mark_torques(axes[0]); axes[0].grid(alpha=0.3)
    if len(eps):
        axes[1].plot(tcs[:len(eps)], np.linalg.norm(eps, axis=1), lw=0.8, color="purple")
    axes[1].set_ylabel("|pos error| [rad]"); axes[1].set_xlabel("wall time [s]")
    axes[1].set_title("position tracking error norm")
    mark_torques(axes[1]); axes[1].grid(alpha=0.3)
    fig.tight_layout(); fig.savefig(f"{OUT}/plot5_controller_state.png", dpi=120); plt.close(fig)

# ---------- 6. Message-rate overview: every stream's inter-arrival on one canvas ----------
fig, axes = plt.subplots(4, 1, figsize=(14, 11), sharex=True)
streams = [
    ("/isaac_joint_states (Isaac tick)", np.array([t - t0 for t, _, _, _ in ijs])),
    ("/isaac_joint_commands (ros2_control 200 Hz)", np.array([t - t0 for t, _, _ in ijc])),
    ("/joint_states (broadcaster)", np.array([t - t0 for t, _, _, _ in js])),
    ("/arm_controller/joint_trajectory (servo out)", np.array([t - t0 for t, _, _ in jt])),
]
for ax, (name, arr) in zip(axes, streams):
    if len(arr) > 2:
        ax.plot(arr[1:], np.diff(arr) * 1000, lw=0.6)
    ax.set_ylabel("gap [ms]"); ax.set_title(name, fontsize=9)
    mark_torques(ax); ax.grid(alpha=0.3)
axes[-1].set_xlabel("wall time [s]")
fig.suptitle("inter-message gaps (wall clock) — stalls show as spikes", y=1.0)
fig.tight_layout(); fig.savefig(f"{OUT}/plot6_stream_rates.png", dpi=120); plt.close(fig)

# ---------- 7. Servo status ----------
if status:
    fig, ax = plt.subplots(figsize=(14, 4))
    st = np.array([t - t0 for t, _ in status])
    sc = np.array([c for _, c in status])
    ax.step(st, sc, where="post", lw=1)
    ax.set_xlabel("wall time [s]"); ax.set_ylabel("status code")
    ax.set_title("/servo_node/status (0=no warning; nonzero = halting/decelerating/singularity)")
    mark_torques(ax); ax.grid(alpha=0.3)
    fig.tight_layout(); fig.savefig(f"{OUT}/plot7_servo_status.png", dpi=120); plt.close(fig)
    codes, counts = np.unique(sc, return_counts=True)
    print("servo status codes:", dict(zip(codes.tolist(), counts.tolist())))

print("saved plots 1-7 to", OUT)
