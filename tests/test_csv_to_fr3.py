"""Offline contract tests; never import ROS or access a robot."""
import csv
import importlib.util
import json
from pathlib import Path
import sys

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("csv_to_fr3", ROOT / "csv_to_fr3.py")
bridge = importlib.util.module_from_spec(spec)
spec.loader.exec_module(bridge)


@pytest.fixture
def config():
    return json.loads((ROOT / "configs/fr3_csv_bridge.json").read_text(encoding="utf-8"))


def source_csv(tmp_path, config, times=(0, 1, 2), theta=(0, 0, 0.7)):
    target = tmp_path / "input.csv"
    q = [0, -0.4, 0, -1.7, 0, 1.5, 0.5]
    with target.open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["time_s", "phase", *config["csv_joints"], config["csv_gripper"]])
        for i, t in enumerate(times):
            pose = q.copy()
            pose[0] = i * 0.1
            w.writerow([t, "approach" if i < 2 else "close", *pose, theta[i]])
    return target


def test_retime_preserves_path_and_gripper_semantics(tmp_path, config):
    src = source_csv(tmp_path, config)
    path = bridge.load_trajectory(src, config)
    assert path["scale"] == pytest.approx(3.5)
    q, grip, phase = bridge.sample_path(path, path["time"][-1])
    assert q[0] == pytest.approx(0.2)
    assert grip == pytest.approx(0.3)  # NOT 0.125 = 1 - theta / 0.8.
    assert phase == "close"
    assert bridge.sample_path(path, 0)[1] == 1


def test_export_resampling_and_deadband(tmp_path, config):
    src = source_csv(tmp_path, config)
    path = bridge.load_trajectory(src, config)
    before = set(sys.modules)
    report = bridge.export_commands(path, config, src, tmp_path / "out")
    assert "rclpy" not in set(sys.modules) - before
    assert report["max_gripper_deadband_error"] < 0.02 + 1e-10
    with (tmp_path / "out/command_stream.csv").open() as f:
        rows = list(csv.DictReader(f))
    t = np.array([float(r["time_s"]) for r in rows])
    q = np.array([[float(r[n]) for n in config["real_joints"]] for r in rows])
    assert np.diff(t) == pytest.approx(np.full(len(t)-1, 0.01))
    assert np.max(np.abs(np.diff(q, axis=0)) / np.diff(t)[:, None]) <= config["max_joint_speed"] + 1e-9
    assert q[-1] == pytest.approx(path["q"][-1])
    assert len(rows) == report["command_rows"]


@pytest.mark.parametrize("times", [(0, 0, 2), (1, 2, 3), (0, 2, 1), (0, float("nan"), 2)])
def test_bad_timestamps_rejected(tmp_path, config, times):
    src = source_csv(tmp_path, config, times=times)
    with pytest.raises(ValueError):
        bridge.load_trajectory(src, config)


def test_wrong_units_and_excess_speed_rejected(tmp_path, config):
    src = source_csv(tmp_path, config, theta=(0, 0, 40))
    with pytest.raises(ValueError, match="knuckle"):
        bridge.load_trajectory(src, config)
    src = source_csv(tmp_path, config)
    with pytest.raises(ValueError, match="time-scale"):
        bridge.load_trajectory(src, config, 1)
    config["joint_max"][0] = 0.15
    with pytest.raises(ValueError, match="joint limits"):
        bridge.load_trajectory(src, config)


def test_feedback_reordered_by_names_not_array_position():
    assert bridge.reorder_state(["j2", "j1"], [2, 1], ["j1", "j2"]) == pytest.approx([1, 2])
    with pytest.raises(KeyError):
        bridge.reorder_state(["j2"], [2], ["j1", "j2"])
    with pytest.raises(ValueError):
        bridge.reorder_state(["j1", "j1"], [1, 2], ["j1"])


def test_gripper_deadband_accumulates_from_last_sent():
    assert not bridge.gripper_due(0.99, 1, 0.02)
    assert bridge.gripper_due(0.98, 1, 0.02)
    assert bridge.gripper_due(1, None, 0.02)


def test_armed_requires_explicit_dds_without_importing_ros(config):
    from types import SimpleNamespace
    with pytest.raises(ValueError, match="dds-config"):
        bridge.run_ros({}, config, SimpleNamespace(dds_config=None))
