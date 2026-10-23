# FR3 真机 CSV 轨迹部署：操作步骤与命令

本文对应 `csv_to_fr3.py`，用于把 Isaac Sim 规划的七轴关节轨迹发送到同事的 FR3 ROS 2 follower 接口。无需启动 VLA 或相机。

**验证状态：离线转换和 10 项测试已通过；尚未进行 ROS 在线联调和真机执行测试。**

## 1. 在哪里执行命令

| 环境 | 用途 |
|---|---|
| Windows PowerShell | 上传部署包、可选的本地离线检查 |
| 同事已部署控制栈的 Linux 主机或容器 | ROS 状态检查和真机轨迹执行 |
| Franka Desk / LABS 界面 | 解锁关节、启用 FCI、确认硬件上线 |

本文的 `用户名@控制电脑IP`、DDS 路径和现场工作空间路径都是需要替换的参数。不要原样输入占位符。

## 2. 上传部署包并解压

在 Windows PowerShell 执行。先修改 `$RobotHost`，如果压缩包改了名称，也修改 `$Bundle`：

```powershell
cd E:\particle_world\camera_loc

$RobotHost = "用户名@控制电脑IP"
$Bundle = ".\dist\fr3_csv_deploy_20261023_104930.zip"

scp $Bundle "${RobotHost}:~/"
ssh $RobotHost
```

登录 Linux 后执行：

```bash
mkdir -p ~/fr3_csv_bundle
unzip ~/fr3_csv_deploy_20261023_104930.zip -d ~/fr3_csv_bundle
cd ~/fr3_csv_bundle/fr3_csv_deploy

ls csv_to_fr3.py configs/fr3_csv_bridge.json outputs/grasp_rrt/trajectory.csv
```

后续终端都在这个目录操作。若 ROS 控制环境在 Docker 中，需把解压目录放进已有挂载目录，或复制进对应容器，然后进入该容器执行后续命令。本文无法确定现场容器名称，不提供猜测的 `docker exec` 或驱动启动命令。

包内主要文件：

```text
fr3_csv_deploy/
├── csv_to_fr3.py
├── configs/fr3_csv_bridge.json
├── outputs/grasp_rrt/trajectory.csv
├── outputs/grasp_rrt/trajectory.json
├── outputs/real_robot_bridge/command_stream.csv
├── outputs/real_robot_bridge/validation.json
├── REAL_ROBOT_CSV.md
├── REAL_ROBOT_DEPLOY_STEPS.md
├── README.md
└── MANIFEST.json
```

## 3. 让机器人和已有控制栈上线

按照同事现场的方式启动机器人驱动、coordinator、follower 和夹爪桥。部署仓库没有包含完整的底层启动环境，以下硬件顺序来自同事文档：

1. 在 Franka Desk / LABS 界面解锁关节（Unlock Joints）。
2. 启用 Start FCI，并确认状态 Online。
3. 如果机器人驱动启动时 FCI 没有开启，按现场流程重新启动驱动。

不要同时运行 VLA 或其他发布同一关节目标话题的程序。物理急停需要可用；后文的软件停止服务不能替代物理急停。

## 4. 加载 ROS 和 DDS 环境

在已有控制栈的 Linux 环境中执行：

```bash
source /opt/ros/jazzy/setup.bash

export ROS_DOMAIN_ID=100
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

如果现场控制栈还要求加载工作空间，请另外执行其实际的 `install/setup.bash`，例如下面这条需要先替换路径：

```bash
# source /实际控制栈工作空间/install/setup.bash
```

指定本机使用的 CycloneDDS XML。以下是同事文档里的路径示例，必须先确认现场路径和网卡配置：

```bash
export DDS_CONFIG=/workspace/embodied-data-lab-develop/deployments/fr3_duo_example/cyclonedds.xml

test -f "$DDS_CONFIG" && echo "DDS file exists"
export CYCLONEDDS_URI="file://$DDS_CONFIG"
```

若没有输出 `DDS file exists`，先修正路径。即便文件存在，XML 内的网卡名/IP 也必须适用于运行本脚本的主机或容器。

检查 Python 依赖：

```bash
python3 -c "import numpy, rclpy; from sensor_msgs.msg import JointState; from std_msgs.msg import Float32; from std_srvs.srv import Trigger; print('dependencies OK')"
```

如果缺少 `rclpy` 或 ROS 消息包，请进入同事已经配置好的 ROS Python 环境，并检查 `source` 操作。不要直接尝试用 `pip install rclpy` 补齐控制栈。

如果只缺 NumPy，可在现场允许安装依赖的 Python 环境中执行：

```bash
python3 -m pip install -r requirements-offline.txt
```

## 5. 只读查看真机状态

```bash
ros2 topic echo /right/franka_robot_state_broadcaster/measured_joint_states \
  --once --qos-reliability best_effort

ros2 topic echo /left/franka_robot_state_broadcaster/measured_joint_states \
  --once --qos-reliability best_effort

ros2 topic echo /right/follower/gripper/joint_states \
  --once --qos-reliability best_effort
```

每条命令收到一条反馈后退出。如果一直没有输出，先排查驱动、ROS domain、DDS 网络和话题名称，不要继续执行运动命令。

查看 coordinator 服务是否存在：

```bash
ros2 service list | grep /right/controller_coordinator/
```

预期包含 `get_ready`、`start_operating`、`stop`。运行回放脚本时由脚本自动握手，不需要提前手动调用 `start_operating`。

## 6. 核对关节映射、起点和现场布置

打开配置：

```bash
nano configs/fr3_csv_bridge.json
```

核对以下字段：

| 字段 | 要求 |
|---|---|
| `real_joints` | 右臂反馈中的真实名称，按物理 joint1 到 joint7 排列 |
| `passive_joints` | 左臂反馈中的真实名称，按物理 joint1 到 joint7 排列 |
| `joint_min` / `joint_max` | 与现场机型和实际软限位一致 |
| `passive_expected` | 与规划时被动臂姿态一致，不能为绕过检查而填任意现场值 |

反馈数组顺序不一定等于 joint1 到 joint7，需按名称对应。还需要确认真实关节零位和正方向与模型一致。

当前轨迹的右臂起点约为：

```text
[0.0, -0.4, 0.0, -1.7, 0.0, 1.5, 0.5] rad
```

当前配置的左臂预期姿态同上。右臂每轴起点允许误差为 0.01 rad，左臂姿态允许误差为 0.05 rad，右夹爪初始应处于与 CSV 匹配的张开状态。

**脚本不提供从任意现场姿态移动到 CSV 起点的规划。**若起点不匹配，需要通过现场已验证的方式到达规划起点，或从真实当前状态重新规划；不能仅修改 CSV 第一行、关节映射或放大容差绕过检查。

当前轨迹对应场景副本中移动后的红块中心：

```text
[-0.30, 0.48, 0.03] m（仿真世界坐标）
```

真机机器人安装、桌面、红块和另一条臂需要对应规划场景。这个仿真坐标不能未经对齐就当作现场世界坐标。当前还未完成真实 TCP、接触和抓取效果验证。

## 7. 先做离线检查，不运动机器人

```bash
python3 csv_to_fr3.py \
  --csv outputs/grasp_rrt/trajectory.csv \
  --config configs/fr3_csv_bridge.json \
  --out outputs/real_robot_bridge
```

查看报告：

```bash
cat outputs/real_robot_bridge/validation.json
```

默认配置下预期约为：

```text
原轨迹时长：22.2167 s
时间放慢倍率：4.37356
导出指令时长：97.17 s
导出指令行数：9718
发布频率：100 Hz
```

此命令没有 `--armed`，不会导入 ROS、连接机器人或下发动作。离线报告不代表真机起点和环境检查通过。

## 8. 真机执行命令

**本步骤会控制右臂及右夹爪。**确认前面的状态、映射、起点和环境核对已完成后，在同一个 ROS 终端执行：

```bash
python3 csv_to_fr3.py \
  --csv outputs/grasp_rrt/trajectory.csv \
  --config configs/fr3_csv_bridge.json \
  --out outputs/real_robot_bridge \
  --dds-config "$DDS_CONFIG" \
  --domain-id 100 \
  --armed
```

输入仍是原始 `outputs/grasp_rrt/trajectory.csv`，不要把导出的 `command_stream.csv` 当作输入。

脚本会依次检查反馈与起点，执行 coordinator 握手，等待 FOLLOWING，再发送轨迹。每个阶段边界等待关节反馈收敛，下降到位后再进入夹爪闭合阶段。

在线耗时会大于离线轨迹时长，因为还包含同步和阶段到位等待。脚本不自动回初始姿态，也不自动张开夹爪；执行完成不等于已经验证抓取成功。

## 9. 查看执行结果

```bash
cat outputs/real_robot_bridge/live_status.json

head -n 2 outputs/real_robot_bridge/live_feedback.csv
tail -n 5 outputs/real_robot_bridge/live_feedback.csv
```

| 文件 | 内容 |
|---|---|
| `validation.json` | 离线转换报告、配置和原 CSV 校验值 |
| `command_stream.csv` | 离线生成的 100 Hz 指令参考 |
| `live_status.json` | 最近一次在线尝试的结果或错误 |
| `live_feedback.csv` | 在线轨迹目标、实测关节、误差和夹爪反馈 |

若在建立在线记录前失败，可能没有新的反馈 CSV；请结合 `live_status.json` 和文件修改时间判断，不要把上一次运行记录误认为本次结果。需要保留多次记录时，每次指定不同的 `--out` 目录。

## 10. 停止与常见报错

运行终端按 `Ctrl+C`，脚本会尝试释放控制器，失败时尝试 coordinator stop。需要从第二个、同样已加载 ROS/DDS 环境的终端请求软件停止时，可以执行：

```bash
ros2 service call /right/controller_coordinator/stop std_srvs/srv/Trigger '{}'
```

软件停止受网络和控制栈状态影响，不替代物理急停。异常退出后先检查机器人当前位置，不要直接重复执行或自动续播。

| 报错或现象 | 处理方向 |
|---|---|
| `No module named rclpy` | 检查 ROS source、Python 版本及现场容器/环境 |
| `Missing or stale feedback` | 检查对应 arm/passive/grip 话题是否持续出数 |
| `Live arm does not match CSV first point` | 起点不同；重新规划或用现场已验证方式到达起点 |
| `Passive arm differs from planned collision scene` | 左臂姿态与碰撞模型不一致 |
| `Initial gripper does not match CSV` | 起始夹爪状态不匹配，脚本拒绝自动张开 |
| 关节名相关错误 | 核对配置与实际 JointState 的 name 字段 |
| `Controller not FOLLOWING` / 握手超时 | 检查驱动、FCI、coordinator 和 follower 状态 |
| `Tracking error` | 停下检查跟踪滞后、控制器状态或机械阻挡，不直接提高阈值 |

更完整的接口与实现说明见同目录 `REAL_ROBOT_CSV.md`。
