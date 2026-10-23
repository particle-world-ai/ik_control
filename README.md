# FR3 CSV trajectory deployment bundle

This bundle contains the independent CSV replay bridge, configuration, original
planned joint trajectory, planning metadata, offline conversion output, and tests.
The colleague's full VLA repository, Isaac Sim, models and robot drivers are NOT
required files for this bridge and are not included.

Offline checks passed (10 tests). No real robot or ROS online test was performed.
The included validation report records the original Windows export location;
those absolute paths are provenance, not runtime dependencies. Planning metadata
also contains source-scene paths for reference; USD assets are not bundled.

## Offline check (Linux, from this directory)
python3 -m pip install -r requirements-offline.txt
python3 csv_to_fr3.py --csv outputs/grasp_rrt/trajectory.csv

Optional tests:
python3 -m pip install pytest
python3 -m pytest tests/test_csv_to_fr3.py -q

## Real robot (existing lab ROS 2 environment)
Read REAL_ROBOT_CSV.md before execution. Verify real joint names, joint zero/sign,
limits, matching initial state, passive-arm pose, and matching physical scene.
This trajectory uses the MOVED simulation object position [-0.30, 0.48, 0.03] m.
Do not change CSV first row or increase start tolerance to bypass start mismatch.

source /opt/ros/jazzy/setup.bash
# Also source the site's controller workspace if required.
export DDS_CONFIG=/absolute/path/to/this_hosts_cyclonedds.xml
export ROS_DOMAIN_ID=100
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$DDS_CONFIG"

# This command DOES operate hardware:
python3 csv_to_fr3.py --csv outputs/grasp_rrt/trajectory.csv --config configs/fr3_csv_bridge.json --out outputs/real_robot_bridge --dds-config "$DDS_CONFIG" --domain-id 100 --armed

Requires installed rclpy, sensor_msgs, std_msgs, std_srvs, CycloneDDS RMW,
and the already running site robot/follower/coordinator/gripper services.
NumPy requirements do not install that external control stack.

Outputs live_feedback.csv and live_status.json on a live attempt. No automatic
move-to-start, return-home or gripper reopening. Ctrl+C requests software cleanup;
it is not a substitute for the physical emergency stop.

MANIFEST.json contains the SHA-256 of every bundled payload file.
