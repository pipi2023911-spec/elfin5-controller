# 基本使用

适用环境：Ubuntu 18.04、ROS Melodic、Python 2.7。接口定义见[当前接口](interfaces.md)。默认配置只用于观察和影子计算；现场适配、限值和停止验证完成后才可人工放行。

## 1. 编译与离线检查

将本仓库和设备对应的厂家 ROS 包放在同一 catkin 工作空间；避免重复的 `elfin5_qp_controller` 包。以 `~/elfin_ws` 为例：

```bash
source /opt/ros/melodic/setup.bash
mkdir -p ~/elfin_ws/src
cd ~/elfin_ws/src
git clone https://github.com/pipi2023911-spec/elfin5-controller.git
# 将已核对版本的厂家 ROS 包放入 src 后继续
cd ~/elfin_ws
rosdep install --from-paths src --ignore-src -r -y
catkin_make
source devel/setup.bash
rospack find elfin5_qp_controller
```

在未连接真机、与真机 ROS master 隔离的环境运行测试：

```bash
cd ~/elfin_ws
catkin_make run_tests_elfin5_qp_controller
catkin_test_results build/test_results
```

上述测试包含纯 C++ 检查、Python 模型/误差检查和合成数据的 ROS 接线测试。接线夹具新增了真实 monitor 与 robot_state_publisher，检查平移/旋转后的 TCP、正确的根 TF 及竞争发布检测；这些 ROS 用例须在 Melodic 环境执行。不要将接线测试节点直接运行到真机 master。仅运行不依赖 ROS 的 Python 检查可使用 `python "$(rospack find elfin5_qp_controller)/test/test_real_robot_tf_and_error.py" -v`。

仿真入口为 `roslaunch elfin5_qp_controller elfin5_qp_gazebo.launch`，仅用于隔离的仿真环境。

## 2. 现场只读检查与 RViz

按设备已经确认的厂家流程连接网口并准备驱动，先核对网卡、标定、轴顺序和控制模式；不能仅凭 RJ45 接口认定使用普通 IP 网络。本包不启动 EtherCAT 主站、不加载或切换厂家控制器、不使能电机。

厂家源码中 `automatic_recognition` 默认开启，需核对实际配置及启动行为；加载 JTC 不等于启动 JTC。先确认停止接口和独立硬件看门狗，再进入控制实验。

```bash
rosservice call /controller_manager/list_controllers "{}"
rostopic echo -n 1 /joint_states
rostopic echo -n 1 /elfin_arm_controller/state
rostopic hz /joint_states
rostopic hz /elfin_arm_controller/state
rostopic echo -n 1 /base_pose
rostopic echo -n 1 /target_point
```

真机使用真实时钟（`/use_sim_time=false`，未设置时默认为 false）。基座位姿和目标均为 absolute_world 坐标，并持续提供有效源时间戳；各设备时钟须同步。清理仿真反馈、竞争的轨迹/action 发布源，以及 QP 旧版本的模型发布器。保留厂家 bringup 的唯一 robot_state_publisher，不删除其 world_base 固定关系。

在已确认厂家启动行为和现场条件后，先运行厂家的模型入口，再运行本包入口（两个终端都先加载工作空间）：

```bash
# 终端一：完整厂家模型和唯一的模型 TF 发布器
roslaunch elfin_robot_bringup elfin5_bringup.launch
```

已核对的 `elfin_robot-melodic_ethercat` 中，`elfin5_bringup.launch` 只加载 URDF 和启动 robot_state_publisher，本身不启动 EtherCAT 驱动或 JTC。真实反馈与控制器仍须由现场已核验的硬件入口提供；不要把模型显示成功当作驱动已运行。

确认厂家 `/robot_description` 已加载后，在终端二启动：

```bash
rosparam get /robot_description | head -n 5
roslaunch elfin5_qp_controller elfin5_qp_real_robot.launch
```

从旧版更新时，先退出旧 QP launch。旧站点配置请同步设置以下四项；基座及目标的坐标源也要使用固定的 `absolute_world`，不要向它们发送模型内随动的 `world` 坐标：

```yaml
bridge:
  world_frame: absolute_world
monitor:
  world_frame: absolute_world
  model_root_frame: world
qp_candidate:
  absolute_frame: absolute_world
```

这些是对原站点 YAML 的合并修改，其他限值和保护配置保留。默认 pose 模式会把基座位姿发布为 `absolute_world → world`；模型内部 TF 全部来自厂家 bringup。


若定位系统已经发布动态 `absolute_world → world`，改用 `base_tf_mode:=external`，仍须提供 base_pose。两种 TF 模式二选一；pose 模式检测到重复发布者后需整理 TF 发布源并重启。

RViz 默认以 absolute_world 为固定坐标系，显示真实反馈驱动的模型、目标、估计 TCP 和误差。观察 `/elfin5_real/bridge/state`、`/elfin5_real/monitor/tracking_state`，并核对 TF、关节方向和现场几何。显示正常不能代替硬件状态或停止确认；监视器/RViz 失效也不会单独停止控制。

只读核对完整 TF 链：

```bash
rosrun tf tf_echo absolute_world world
rosrun tf tf_echo world elfin_base_link
rosrun tf tf_echo absolute_world elfin_end_link
rosnode info /robot_state_publisher
```

第二条应始终是零平移、单位旋转；第一条应与真实 base_pose 一致。基座移动时厂家 world 和机器人模型应整体在 RViz 中移动。external 模式要求同一条动态根变换，不能直接另发 absolute_world 到 elfin_base_link，造成基座双父关系。没有定位输入时应排查定位来源，不用固定零位姿冒充移动基座反馈。

## 3. 记录数据与影子计算

另开终端加载工作空间后，先记录数据：

```bash
mkdir -p "$HOME/elfin5_real_logs"
rosbag record -o "$HOME/elfin5_real_logs/elfin5" \
  /joint_states /base_pose /target_point /tf /tf_static \
  /elfin_arm_controller/state /elfin_arm_controller/command \
  /elfin5_hardware/status \
  /elfin5_real/candidate_trajectory /elfin5_real/qp_candidate/state \
  /elfin5_real/bridge/state /elfin5_real/monitor/current_point \
  /elfin5_real/monitor/tracking_state /elfin5_real/monitor/joint_tracking_error
```

保存代码版本、配置、定位来源、负载与工况。可以用 `rosparam dump "$HOME/elfin5_real_logs/parameters.yaml" /elfin5_real` 保存参数。

只开启 QP 候选计算，保持网关未放行：

```bash
rosservice call /elfin5_real/qp_candidate/enable "data: true"
rostopic echo /elfin5_real/qp_candidate/state
rostopic hz /elfin5_real/candidate_trajectory
```

这一步可检查输入、求解状态和记录；候选包括异常分支的保持命令，都不会因影子计算直接发到真实 JTC。

## 4. 站点配置与人工放行

复制 [elfin5_real_robot.yaml](../config/elfin5_real_robot.yaml) 为站点配置，填写已核实的六轴位置上下界、速度/加速度限值、端点最大步长、设备 ID 及话题/服务。位置限制不得越出模型范围。

站点还须提供[硬件诊断和停止契约](interfaces.md)：`/elfin5_hardware/status` 与 `/elfin5_site/stop`。本仓库不包含厂家专用适配器。停止服务回执不等同于物理静止；禁止用固定 OK 诊断、空停止服务或放宽阈值绕过保护。

完成硬件互锁、控制器插值、限值、停止及断网看门狗核验后，才设置 `bridge.site_review_complete=true`。该开关只是审查记录，不能自动证明上述条件。参数修改后须先停止，再重启加载：

```bash
roslaunch elfin5_qp_controller elfin5_qp_real_robot.launch \
  hardware_config:="$HOME/elfin5_site/real_robot.yaml" base_tf_mode:=pose
```

按现场实际选择 TF 模式。重新启用影子计算，确认网关为 READY。在现场人员确认的安全测试条件下，显式放行：

```bash
rosservice call /elfin5_real/bridge/enable "data: true"
```

需服务返回 `success=true` 且状态成为 ACTIVE。首次使用低速、小范围可达目标，先静止基座，再逐步测试移动基座。当前算法没有基座运动速度前馈；到点容差内保持关节角，基座运动仍可能引入 TCP 误差。

## 5. 停止与故障处理

```bash
rosservice call /elfin5_real/bridge/enable "data: false"
rosservice call /elfin5_real/qp_candidate/enable "data: false"
```

正常关闭网关会禁止后续轨迹并请求站点停止。等待停止回执、确认机械臂实际停止后，再退出 launch 和录包；Ctrl+C、断网或停止发命令不能替代停止流程。正常关闭不需 reset，恢复运动仍需人工 enable。

FAULT_LATCHED 时先处理实际故障并确认停机。停止服务失败会禁止 reset，处理停止接口后显式再次调用 bridge/enable false 才重试；成功后再调用 `/elfin5_real/bridge/reset`。reset 不使能硬件、不恢复运动。

遇到阻断，记录状态原因、对应代码位置、观测值与阈值，区分安全/算法必要检查和冗余检查；确认原因后仅修改相关项。没有依据时保留保护。

## 6. 误差曲线

录包结束后生成新的输出目录：

```bash
rosrun elfin5_qp_controller plot_real_robot_error.py /path/to/experiment.bag \
  --output /path/to/new_error_result
```

输出末端误差范数 `tcp_error.png`（mm）、六轴有符号误差 `joint_tracking_error.png`（deg），以及 `tcp_error.csv`、`joint_error.csv`、`events.csv`、`statistics.json`；无有效数据时不生成对应曲线。末端误差不拆分 XYZ，关节误差从同一 JTC 消息的 desired−actual 计算。

可选 `--active-only` 只统计 ACTIVE 区间；`--tcp-tolerance-mm` 和 `--joint-tolerance-deg`（六个数）用于事先定义的分析容差。话题改名时同步传入脚本的话题参数。模型估计的 TCP 误差不能代替外部测量的绝对精度。

## 当前验证范围与待核实参数

本次 14 项 Python 模型/误差/启动配置检查通过。19 项纯 C++ 命令检查断言通过；提取实际网关状态方法及 monitor 的 onTF/onBase 回调，在时钟、服务和消息桩下编译运行通过，覆盖人工放行/停止/复位、故障锁存、正常厂家 TF、根竞争和 external 模式不广播。这些桩测试不等于 ROS 节点集成测试。完整 ROS 编译、ROS 隔离测试及真机实验尚未完成，不能以源码检查替代这些验证。

截至当前，只有厂家源码，尚无 joint_states、JTC state、base_pose 的实测发布周期/最大延迟，以及厂家停止/看门狗说明。以下保留值均未通过现场核验，不是已确认的安全阈值：

| 参数 | 当前默认值 |
|---|---|
| bridge/monitor `input_timeout` | 0.3 s |
| bridge/monitor `max_input_skew` | 0.08 s |
| bridge `candidate_max_age` | 0.02 s |
| bridge `candidate_timeout` | 0.15 s |
| bridge `min_horizon` / `max_horizon` | 0.02 / 0.15 s |
| monitor 外部 TF 位置/角度容差 | 0.001 m / 0.001 rad |

还需核实现场限值、定位精度与时钟、硬件互锁来源、停止响应和独立看门狗行为。取得实测与厂家依据后再局部调整配置；保留现有保护，不为消除阻断任意放宽数值。
