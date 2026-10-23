# 从 Isaac 轨迹到 FR3 真机控制

这次阅读的是本地 `E:\particle_world\pw-xpolicy-deploy-main`。当前新增代码独立放在 camera_loc，未改同事仓库、原规划脚本及原 CSV。已完成离线导出和自动检查，未连接或运动真机；ROS 在线分支还需要在实验室环境联调。

## 同事代码如何控制机械臂

主要看 `DeployLab/Franka_Duo/dual_arm_policy_bridge.py`，不是旧的 `fr3_right_arm_bridge.py` 六轴映射版本。

1. 从 OMS 的 RTSP 相机读取图像，从 ROS 2 读取实测关节位置。
2. `run_pwjepa_infer.py` / 模型适配器处理观测，VLA 返回动作块。当前 FR3 原生接口为 16 维：左臂 7 轴 + 左夹爪 + 右臂 7 轴 + 右夹爪。默认关节输出是绝对弧度值，也有可选 delta 模式。
3. 对动作进行插值、限速和范围检查，以 100 Hz 发布 ROS 2 指令。关节位置经过外部 follower 控制器的低通滤波；仓库说明该滤波有明显滞后。
4. 通过 coordinator 服务完成 `get_ready → READY → start_operating → SYNCING → FOLLOWING`，然后执行。相机和 VLA 负责产生目标，真正伺服及 Franka 硬件接口由外部 LABS/ROS 控制栈负责，不在这个仓库里。

| 用途 | 右臂接口 | 消息/单位 |
|---|---|---|
| 关节目标 | `/right/follower/gello/joint_states` | `sensor_msgs/JointState`，7 个绝对关节角，rad |
| 关节反馈 | `/right/franka_robot_state_broadcaster/measured_joint_states` | `sensor_msgs/JointState` |
| 夹爪目标 | `/right/follower/gripper/gripper_client/target_gripper_width_percent` | `std_msgs/Float32`，0..1，数值越大越张开 |
| 夹爪反馈 | `/right/follower/gripper/joint_states` | 第一个位置值为 knuckle 角，rad |
| 状态 | `/right/controller_coordinator/state`、`/right/joint_follower_controller/state` | `std_msgs/String` |
| 接管/释放 | `/right/controller_coordinator/{get_ready,start_operating,stop}` | `std_srvs/Trigger` |

平台夹爪的换算以 `right_gripper_test.py:angle_to_percent` 为准：`width_percent = clip(1 - knuckle_rad, 0, 1)`。不是把角度除以 0.8。原 CSV 的 0 rad 对应 1.0，0.7 rad 对应 0.3。客户端有约 0.02 的命令死区，新脚本按距离上次发送值的累计变化发送，导出文件同时记录请求值和实际发送值；末端可能有不足 0.02 的宽度命令差。

## 新脚本做什么

`csv_to_fr3.py` 直接用原轨迹替代 VLA 动作，保留七轴绝对角语义。`configs/fr3_csv_bridge.json` 显式配置 CSV 列名到真机关节名的对应关系、平台关节范围、限速及反馈检查。

- 默认只离线读取 CSV，检查时间、数值、关节范围并保留路径几何，统一放慢时间、重采样为 100 Hz。越界会报错，不通过裁剪改变路径。
- 只有显式指定 `--armed` 才导入 ROS、连接控制栈及发指令。只发布选中的右臂及其夹爪，不给另一条臂下发动作。
- 在线时按关节名称重排反馈，要求实测起点与 CSV 首点每轴差不超过 0.01 rad；起始夹爪也必须匹配。不自动从任意现场姿态移动到起点。
- 另一条臂只订阅检查：必须与仿真碰撞场景中的预期位置接近，并保持新鲜反馈。
- 接管后等待 coordinator 和 follower 均为 FOLLOWING；执行中检查反馈新鲜度、状态、跟踪误差和发送超时。
- 每个阶段边界等待关节收敛再继续，避免尚未到达抓取点就闭合。夹住物体后夹爪未达到空载角度是正常现象，因此不强求夹爪反馈等于 0.7 rad。
- 结束或异常时请求释放控制器；不自动回初始姿态、不自动张开夹爪。coordinator 的 stop 是软件服务，不能替代物理急停。释放后机器人保持行为仍取决于外部控制器。

## 本地离线运行

在 PowerShell 中：

```powershell
cd E:\particle_world\camera_loc
.\.venv\Scripts\python.exe .\csv_to_fr3.py --csv .\outputs\grasp_rrt\trajectory.csv
```

默认结果：`outputs/real_robot_bridge/command_stream.csv`、`validation.json`。CSV 含时间、阶段、七轴目标、夹爪请求值、发送标记和上次发送宽度。报告只描述离线检查，不代表真机执行通过。

当前轨迹 1,334 个原始点、22.2167 s；默认限速自动计算时间倍率约 4.37356，导出 9,718 点、97.17 s。最大关节目标速度约 0.07981 rad/s。在线还会增加阶段到位等待、初始同步时间，实际耗时更长。

## 实验室 ROS 2 主机运行

在同事实测使用的 ROS 2 / CycloneDDS 环境中放置脚本、配置及原 CSV。无需安装 VLA 模型，但需要现有机器人驱动、coordinator、follower、夹爪桥等外部服务已经运行。Windows 上离线导出不会启动这些服务。

先用实测话题核对 `real_joints` 和 `passive_joints`，配置中的名称只是按仓库命名约定填写。还要核对关节零位/正方向、机器人和夹爪型号、实际软限位及被动臂位置。DDS XML 必须指向运行脚本这台主机的真实网卡，不能直接照搬同事 XML 内固定的主机地址。

下面是 Linux ROS 2 主机上的执行模板，指定 `--armed` 会控制真机：

```bash
source /opt/ros/jazzy/setup.bash
# 同时 source 现场控制栈要求的工作空间。
python3 csv_to_fr3.py \
  --csv outputs/grasp_rrt/trajectory.csv \
  --config configs/fr3_csv_bridge.json \
  --out outputs/real_robot_bridge \
  --dds-config /absolute/path/to/this_host_cyclonedds.xml \
  --domain-id 100 \
  --armed
```

依赖 `numpy`、`rclpy`、`sensor_msgs`、`std_msgs`、`std_srvs` 和 CycloneDDS RMW。不提供自建底层机器人驱动。在线输出 `live_feedback.csv` 和 `live_status.json`，反馈文件包含实测/目标关节及误差；软件异常后文件可能只有已执行部分，不能据此自动续播。

## 当前轨迹能否直接抓真机上的红块

接口和数据维度可以适配，但当前没有完成实机可用性验证。

这条轨迹来自场景副本：红块初始中心已从约 `[-0.3645, 0.1638, 0.03] m` 移到 `[-0.30, 0.48, 0.03] m`。直接回放关节角不再使用相机外参，但真机的安装、红块、桌面和另一条臂必须对应这一规划场景。真实 TCP、夹爪行程/力度、模型零位及接触行为尚未校验。不能把 Isaac 中抬起成功解释成真机抓取已验证。

本脚本只有状态与关节误差监控，没有现场碰撞感知、硬实时保证、接触力控制或抓取成功检测。follower 滤波还可能改变实际末端路径；阶段等待能解决动作先后顺序，不能证明整条实测路径无碰撞。

验证命令：`.\.venv\Scripts\python.exe -m pytest tests\test_csv_to_fr3.py -q`。覆盖离线转换、单位、限速、非法输入、死区和名称映射；不包含真机或 ROS 环境测试。
