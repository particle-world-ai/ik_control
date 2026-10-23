"""Isaac CSV -> colleague's FR3 ROS 2 follower. Offline by default; no VLA needed."""
from __future__ import annotations

import argparse
import bisect
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import time

import numpy as np

ROOT = Path(__file__).resolve().parent


def load_trajectory(path, config, time_scale=None):
    """Reject invalid paths; retime rather than clip joint targets."""
    names = config["csv_joints"]
    if len(names) != 7 or len(set(names)) != 7:
        raise ValueError("csv_joints must contain seven distinct names")
    for key in ("real_joints", "passive_joints"):
        if len(config[key]) != 7 or len(set(config[key])) != 7:
            raise ValueError(f"{key} must contain seven distinct names")
    for key in ("control_hz", "max_joint_speed", "max_gripper_speed", "gripper_deadband",
                "start_tolerance", "tracking_abort", "settle_tolerance", "settle_timeout_s",
                "state_timeout_s", "passive_tolerance"):
        if not math.isfinite(config[key]) or config[key] <= 0:
            raise ValueError(f"Invalid positive setting: {key}")
    if (config["side"] not in ("left", "right") or config["passive_side"] not in ("left", "right")
            or config["passive_side"] == config["side"]):
        raise ValueError("Invalid active/passive sides")
    passive = np.asarray(config["passive_expected"], dtype=float)
    if passive.shape != (7,) or not np.isfinite(passive).all():
        raise ValueError("Invalid passive arm pose")
    with open(path, newline="", encoding="utf-8-sig") as f:
        reader = csv.DictReader(f)
        fields = reader.fieldnames or []
        required = ["time_s", "phase", *names, config["csv_gripper"]]
        if len(fields) != len(set(fields)) or not set(required).issubset(fields):
            raise ValueError("Missing or duplicate CSV columns")
        rows = list(reader)
    if len(rows) < 2:
        raise ValueError("Trajectory needs at least two rows")
    t = np.array([float(r["time_s"]) for r in rows])
    q = np.array([[float(r[n]) for n in names] for r in rows])
    theta = np.array([float(r[config["csv_gripper"]]) for r in rows])
    phases = [r["phase"].strip() for r in rows]
    if not all(np.isfinite(a).all() for a in (t, q, theta)):
        raise ValueError("CSV contains non-finite numbers")
    if abs(t[0]) > 1e-9 or np.any(np.diff(t) <= 0) or not all(phases):
        raise ValueError("Require t[0]=0, strictly increasing timestamps, nonempty phases")
    lo, hi = np.asarray(config["joint_min"]), np.asarray(config["joint_max"])
    if lo.shape != (7,) or hi.shape != (7,) or not np.isfinite([lo, hi]).all() or np.any(lo >= hi):
        raise ValueError("Invalid joint limit vectors")
    if np.any(q < lo) or np.any(q > hi):
        raise ValueError("Trajectory exceeds deployed joint limits; no automatic clipping")
    if np.any(theta < 0) or np.any(theta > 0.8):
        raise ValueError("Expected Robotiq knuckle angles in [0,0.8] rad")
    # This is the deployment client's actual mapping, NOT 1 - theta / 0.8.
    grip = 1.0 - theta
    dt = np.diff(t)
    speed = float(np.max(np.abs(np.diff(q, axis=0)) / dt[:, None]))
    grip_speed = float(np.max(np.abs(np.diff(grip)) / dt))
    minimum_scale = max(1.0, speed / config["max_joint_speed"], grip_speed / config["max_gripper_speed"])
    scale = minimum_scale if time_scale is None else float(time_scale)
    if not math.isfinite(scale) or scale < minimum_scale - 1e-9:
        raise ValueError(f"time-scale must be >= {minimum_scale:.6f}")
    t *= scale
    return {"time": t, "q": q, "grip": grip, "phase": phases, "scale": scale,
            "original_duration_s": float(t[-1] / scale), "source_max_joint_speed": speed,
            "source_max_gripper_speed": grip_speed}


def sample_path(path, seconds):
    t = path["time"]
    index = min(bisect.bisect_right(t, seconds) - 1, len(t) - 1)
    index = max(0, index)
    q = np.array([np.interp(seconds, t, path["q"][:, j]) for j in range(7)])
    return q, float(np.interp(seconds, t, path["grip"])), path["phase"][index]


def gripper_due(requested, accepted, deadband):
    return accepted is None or abs(requested - accepted) >= deadband - 1e-12


def export_commands(path, config, source, out):
    out.mkdir(parents=True, exist_ok=True)
    last = None
    max_error = 0.0
    count = int(math.ceil(path["time"][-1] * config["control_hz"])) + 1
    with (out / "command_stream.csv").open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["time_s", "phase", *config["real_joints"], "requested_width_percent",
                         "publish_gripper", "last_sent_width_percent"])
        for i in range(count):
            seconds = i / config["control_hz"]
            q, grip, phase = sample_path(path, seconds)
            due = gripper_due(grip, last, config["gripper_deadband"])
            if due:
                last = grip
            max_error = max(max_error, abs(grip - last))
            writer.writerow([seconds, phase, *q, grip, int(due), last])
    report = {
        "mode": "offline_only_no_ROS_import_or_robot_connection",
        "csv": str(Path(source).resolve()), "csv_sha256": hashlib.sha256(Path(source).read_bytes()).hexdigest(),
        "config": config, "original_duration_s": path["original_duration_s"],
        "time_scale": path["scale"], "resampled_duration_s": (count - 1) / config["control_hz"],
        "command_rows": count, "max_joint_speed_rad_s": path["source_max_joint_speed"] / path["scale"],
        "max_width_speed_per_s": path["source_max_gripper_speed"] / path["scale"],
        "max_gripper_deadband_error": max_error, "initial_q_rad": path["q"][0].tolist(),
        "final_q_rad": path["q"][-1].tolist(), "phase_order": list(dict.fromkeys(path["phase"])),
        "limits": ["This report covers offline export only; see live_status.json for any subsequent live attempt",
                   "Real joint names and joint zero/sign must be verified",
                   "Actual scene, TCP, passive arm and initial state must match the plan",
                   "Follower lag changes executed path; this is not a certified collision check",
                   "Gripper state alone does not confirm successful object contact"]}
    (out / "validation.json").write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    return report


def reorder_state(names, values, expected):
    if len(names) != len(values) or len(set(names)) != len(names):
        raise ValueError("Invalid measured JointState names")
    mapping = dict(zip(names, values))
    q = np.array([mapping[n] for n in expected], dtype=float)
    if not np.isfinite(q).all():
        raise ValueError("Invalid measured joint positions")
    return q


def run_ros(path, config, args):
    """Only called with --armed on the lab's ROS 2 host; never from offline export."""
    dds = Path(args.dds_config).resolve() if args.dds_config else None
    if dds is None or not dds.is_file():
        raise ValueError("--armed requires --dds-config for this ROS host's network interface")
    os.environ["RMW_IMPLEMENTATION"] = "rmw_cyclonedds_cpp"
    os.environ["ROS_DOMAIN_ID"] = str(args.domain_id)
    os.environ["CYCLONEDDS_URI"] = dds.as_uri()
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Float32, String
    from std_srvs.srv import Trigger

    rclpy.init(args=[])
    node = rclpy.create_node("csv_fr3_trajectory_bridge")
    side = config["side"]
    state = {}
    follow = {}
    subscriptions = []
    pubs = {}
    touched = False
    last_grip = None
    period = 1.0 / config["control_hz"]
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)

    def receive_arm(msg, key, names):
        try:
            state[key] = (reorder_state(list(msg.name), list(msg.position), names), time.monotonic())
        except (KeyError, ValueError) as exc:
            state["invalid"] = str(exc)

    def receive_grip(msg):
        # Matches the colleague's first-knuckle feedback convention.
        if len(msg.position) and math.isfinite(msg.position[0]) and 0 <= msg.position[0] <= 0.8:
            state["grip"] = (float(msg.position[0]), time.monotonic())
        else:
            state["invalid"] = "Invalid gripper feedback"

    subscriptions.append(node.create_subscription(JointState,
        f"/{side}/franka_robot_state_broadcaster/measured_joint_states",
        lambda m: receive_arm(m, "arm", config["real_joints"]), qos))
    subscriptions.append(node.create_subscription(JointState,
        f"/{config['passive_side']}/franka_robot_state_broadcaster/measured_joint_states",
        lambda m: receive_arm(m, "passive", config["passive_joints"]), qos))
    subscriptions.append(node.create_subscription(JointState,
        f"/{side}/follower/gripper/joint_states", receive_grip, qos))
    for key, controller in (("coordinator", "controller_coordinator"), ("follower", "joint_follower_controller")):
        subscriptions.append(node.create_subscription(String, f"/{side}/{controller}/state",
            lambda m, k=key: follow.__setitem__(k, m.data), latched))

    def spin_until(deadline):
        while time.monotonic() < deadline:
            if not rclpy.ok():
                raise RuntimeError("ROS context stopped")
            rclpy.spin_once(node, timeout_sec=min(0.01, max(0.0, deadline - time.monotonic())))

    def check(require_follow=True):
        if "invalid" in state:
            raise RuntimeError(state["invalid"])
        now = time.monotonic()
        for key in ("arm", "passive", "grip"):
            if key not in state or now - state[key][1] > config["state_timeout_s"]:
                raise RuntimeError(f"Missing or stale feedback: {key}")
        if (np.any(state["arm"][0] < config["joint_min"])
                or np.any(state["arm"][0] > config["joint_max"])):
            raise RuntimeError("Measured arm exceeds configured joint limits")
        if np.max(np.abs(state["passive"][0] - config["passive_expected"])) > config["passive_tolerance"]:
            raise RuntimeError("Passive arm differs from planned collision scene")
        if require_follow and (follow.get("coordinator") != "FOLLOWING" or follow.get("follower") != "FOLLOWING"):
            raise RuntimeError(f"Controller not FOLLOWING: {follow}")

    def send(q, grip=None):
        nonlocal last_grip
        message = JointState()
        message.header.stamp = node.get_clock().now().to_msg()
        message.name = config["real_joints"]
        message.position = [float(v) for v in q]
        pubs["arm"].publish(message)
        if grip is not None and gripper_due(grip, last_grip, config["gripper_deadband"]):
            pubs["grip"].publish(Float32(data=float(grip)))
            last_grip = grip

    def service(action):
        client = node.create_client(Trigger, f"/{side}/controller_coordinator/{action}")
        try:
            if not client.wait_for_service(timeout_sec=3):
                raise RuntimeError(f"Service unavailable: {action}")
            future = client.call_async(Trigger.Request())
            deadline = time.monotonic() + 25
            while not future.done() and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.01)
                # Feed a stationary measured target during controller activation.
                if action != "stop" and "arm" in pubs and "arm" in state:
                    check(require_follow=False)
                    send(state["arm"][0])
            if not future.done() or not future.result() or not future.result().success:
                raise RuntimeError(f"Coordinator service failed: {action}")
        finally:
            node.destroy_client(client)

    def settle(q, grip, label):
        deadline = time.monotonic() + config["settle_timeout_s"]
        stable_since = None
        while time.monotonic() < deadline:
            check()
            error = float(np.max(np.abs(state["arm"][0] - q)))
            if error > config["tracking_abort"]:
                raise RuntimeError(f"Tracking error {error:.4f} rad during {label}")
            if error <= config["settle_tolerance"]:
                stable_since = stable_since or time.monotonic()
                if time.monotonic() - stable_since >= 0.3:
                    return
            else:
                stable_since = None
            send(q, grip)
            spin_until(time.monotonic() + period)
        raise RuntimeError(f"Failed to converge at phase boundary: {label}")

    try:
        deadline = time.monotonic() + 8
        while not all(k in state for k in ("arm", "passive", "grip")) and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        check(require_follow=False)
        if np.max(np.abs(state["arm"][0] - path["q"][0])) > config["start_tolerance"]:
            raise RuntimeError("Live arm does not match CSV first point; no automatic move-to-start")
        if abs(1.0 - state["grip"][0] - path["grip"][0]) > config["gripper_deadband"]:
            raise RuntimeError("Initial gripper does not match CSV; refusing automatic opening")
        initial_hold = state["arm"][0].copy()
        pubs["arm"] = node.create_publisher(JointState, f"/{side}/follower/gello/joint_states", 10)
        pubs["grip"] = node.create_publisher(Float32,
            f"/{side}/follower/gripper/gripper_client/target_gripper_width_percent", 10)
        touched = True  # Cleanup also runs if takeover only partly succeeds.
        service("get_ready")
        if follow.get("coordinator") != "READY":
            spin_until(time.monotonic() + 0.2)
        if follow.get("coordinator") != "READY":
            raise RuntimeError("get_ready did not produce READY")
        service("start_operating")
        deadline = time.monotonic() + 15
        while not (follow.get("coordinator") == follow.get("follower") == "FOLLOWING"):
            if time.monotonic() > deadline:
                raise RuntimeError("Timed out waiting for FOLLOWING")
            check(require_follow=False)
            send(initial_hold)
            spin_until(time.monotonic() + period)
        check()
        # Even this small initial correction is a rate-limited move, not a jump.
        for alpha in np.linspace(0, 1, max(2, int(config["control_hz"]))):
            check()
            send((1-alpha)*initial_hold + alpha*path["q"][0])
            spin_until(time.monotonic() + period)
        settle(path["q"][0], path["grip"][0], "start")
        boundaries = [0] + [i for i in range(1, len(path["phase"]))
                            if path["phase"][i] != path["phase"][i-1]] + [len(path["time"]) - 1]
        trace_path = Path(args.out) / "live_feedback.csv"
        with trace_path.open("w", newline="", encoding="utf-8") as trace:
            writer = csv.writer(trace)
            writer.writerow(["wall_elapsed_s", "path_time_s", "phase", "max_tracking_error_rad",
                             *[f"measured_q{i+1}" for i in range(7)], *[f"target_q{i+1}" for i in range(7)],
                             "requested_width_percent", "last_sent_width_percent", "measured_knuckle_rad"])
            wall_start = time.monotonic()
            for start, end in zip(boundaries[:-1], boundaries[1:]):
                # Stop at the OLD gripper value before crossing each phase boundary.
                # Ensures descent has converged BEFORE closure; no gripper-equality
                # check after closure because the object can prevent full closing.
                begin_t, end_t = path["time"][[start, end]]
                ticks = int(math.ceil((end_t - begin_t) / period))
                epoch = time.monotonic()
                for tick in range(ticks + 1):
                    due = epoch + tick * period
                    spin_until(due)
                    if time.monotonic() - due > 0.1:
                        raise RuntimeError("Publish deadline missed; refusing catch-up burst")
                    check()
                    seconds = min(begin_t + tick * period, end_t)
                    q, grip, phase = sample_path(path, seconds)
                    error = float(np.max(np.abs(state["arm"][0] - q)))
                    if error > config["tracking_abort"]:
                        raise RuntimeError(f"Tracking error {error:.4f} rad")
                    # At segment end use the preceding phase's gripper target.
                    if seconds >= end_t and end > start:
                        grip = float(path["grip"][end - 1])
                    send(q, grip)
                    writer.writerow([time.monotonic()-wall_start, seconds, phase, error,
                                     *state["arm"][0], *q, grip, last_grip, state["grip"][0]])
                settle(path["q"][end], float(path["grip"][max(start, end-1)]), path["phase"][start])
            settle(path["q"][-1], float(path["grip"][-1]), "final")
        print(f"Completed command stream; feedback: {trace_path}. Grasp success is not measured.")
    finally:
        if touched:
            # No automatic home move, gripper reopen, or replay on any exit.
            try:
                service("get_ready")
            except BaseException as exc:
                print(f"Release failed ({exc}); requesting coordinator stop")
                try:
                    service("stop")
                except BaseException as stop_exc:
                    print(f"STOP FAILED: {stop_exc}; use the robot's physical stop")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--csv", type=Path, default=ROOT / "outputs/grasp_rrt/trajectory.csv")
    parser.add_argument("--config", type=Path, default=ROOT / "configs/fr3_csv_bridge.json")
    parser.add_argument("--out", type=Path, default=ROOT / "outputs/real_robot_bridge")
    parser.add_argument("--time-scale", type=float, help="Default: automatic slowdown from speed bounds")
    parser.add_argument("--armed", action="store_true", help="Publish to real hardware (ROS 2 host only)")
    parser.add_argument("--dds-config", type=Path)
    parser.add_argument("--domain-id", type=int, default=100)
    args = parser.parse_args()
    config = json.loads(args.config.read_text(encoding="utf-8-sig"))
    path = load_trajectory(args.csv, config, args.time_scale)
    report = export_commands(path, config, args.csv, args.out)
    print(json.dumps({k: report[k] for k in ("mode", "command_rows", "original_duration_s",
          "time_scale", "resampled_duration_s", "max_joint_speed_rad_s")}, indent=2))
    if args.armed:
        status = {"result": "failed", "csv_sha256": report["csv_sha256"],
                  "config": config, "started_unix_s": time.time()}
        try:
            run_ros(path, config, args)
            status["result"] = "command_stream_completed_grasp_not_verified"
        except BaseException as exc:
            status["error"] = str(exc)
            raise
        finally:
            status["finished_unix_s"] = time.time()
            (args.out / "live_status.json").write_text(json.dumps(status, indent=2), encoding="utf-8")


if __name__ == "__main__":
    main()
