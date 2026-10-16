# 当前接口

本文对应当前 V2 实现。以下名称以 `elfin5_qp_real_robot.launch` 默认配置为准；现场改名时需同步修改配置、录包与绘图参数。

## 坐标与控制量

- 长度为 m，关节位置为 rad，速度为 rad/s，加速度为 rad/s²，时间为 s。
- `absolute_world` 固定；`base_pose` 表示 `elfin_base_link` 在 `absolute_world` 中的位姿，即把基座坐标变换到世界坐标的 `T_absolute_world_base`。
- `target_point` 为 `absolute_world` 中的目标 XYZ；末端为 `elfin_end_link`，不约束姿态。
- 默认六轴为 `elfin_joint1` 至 `elfin_joint6`。消息按名称匹配，限值数组按 `joint_names` 顺序填写。
- 输入须具有有效、非零、递增的源时间戳和有限数值；基座四元数须符合单位四元数检查。基座与目标的 `header.frame_id` 为 `absolute_world`。

## 控制链与节点

```text
/joint_states、/base_pose、/target_point
    → bridge 输入检查 → /elfin5_real/validated/*
    → qp_candidate → /elfin5_real/candidate_trajectory
    → bridge 放行和命令检查 → /elfin_arm_controller/command
    → 厂家位置 JTC → RobotHW → EtherCAT
```

`/elfin5_real/bridge` 管理真实命令放行；`/elfin5_real/qp_candidate` 复用现有 QP；`/elfin5_real/monitor` 只读计算误差和显示。QP launch 只启动这三个节点和可选的 `/elfin5_real/rviz`，不再启动模型发布器。监视与 RViz 不参与运动准入。QP 或 bridge 退出会触发 launch 关闭，但进程退出不能代替站点停止及硬件看门狗。

## 输入话题

| 默认话题 | 类型 | 来源与用途 |
|---|---|---|
| `/joint_states` | `sensor_msgs/JointState` | 厂家真实关节反馈，须包含六轴位置 |
| `/base_pose` | `geometry_msgs/PoseStamped` | 定位系统提供基座在 absolute_world 中的位姿 |
| `/target_point` | `geometry_msgs/PointStamped` | 上游提供 absolute_world 中的目标点 |
| `/elfin_arm_controller/state` | `control_msgs/JointTrajectoryControllerState` | JTC 提供六轴 desired 位置/速度与 actual 位置，供命令检查和关节误差计算 |
| `/elfin5_hardware/status` | `diagnostic_msgs/DiagnosticArray` | 现场适配器汇总硬件互锁；本包未实现厂家专用适配器 |
| `/elfin5_real/qp_candidate/state` | `diagnostic_msgs/DiagnosticArray` | QP 状态，供 bridge 判断计算是否正常 |

网关对这些输入检查格式、源时间和接收时效，并限制关节反馈与基座、JTC 反馈的时间差。固定目标也需要上游持续发布有效的新样本；不得为失效数据重打时间戳。

硬件诊断须恰好有一个 `name=elfin5_hardware` 的匹配项，`hardware_id` 与非空配置值一致，且 `level=OK`。现场适配器只有在使能、位置模式、驱动故障、急停、看门狗及执行权限等实际条件满足时才能报告 OK；未知或不满足应报告非 OK。该话题应持续发布、使用真实时间戳且非锁存；不能用固定 OK 心跳代替。网关不直接读取 EtherCAT WKC、OP 状态或驱动故障字。

## 输出与诊断

| 默认话题 | 类型 | 含义 |
|---|---|---|
| `/elfin5_real/validated/joint_states` | `sensor_msgs/JointState` | 检查通过的关节反馈，供 QP 使用；厂家模型发布器直接使用真实 /joint_states |
| `/elfin5_real/validated/base_pose` | `geometry_msgs/PoseStamped` | 检查通过的基座输入 |
| `/elfin5_real/validated/target_point` | `geometry_msgs/PointStamped` | 检查通过的目标输入 |
| `/elfin5_real/candidate_trajectory` | `trajectory_msgs/JointTrajectory` | QP 候选，尚未代表实际下发 |
| `/elfin_arm_controller/command` | `trajectory_msgs/JointTrajectory` | 仅 ACTIVE 且命令检查通过时由 bridge 原样下发 |
| `/elfin5_real/bridge/state` | `std_msgs/String` | 锁存发布的 `状态: 原因`，无消息头时间戳 |
| `/elfin5_real/qp_candidate/state` | `diagnostic_msgs/DiagnosticArray` | QP 状态和求解诊断 |
| `/elfin5_real/qp_candidate/current_point` | `geometry_msgs/PointStamped` | QP 正运动学估计的 absolute_world 末端位置 |
| `/elfin5_real/qp_candidate/position_error` | `std_msgs/Float64` | QP 末端位置误差，m |
| `/elfin5_real/qp_candidate/min_singular_value` | `std_msgs/Float64` | 平移 Jacobian 的最小奇异值 |

候选轨迹只接受单点、六轴位置和速度，不接受额外加速度或力矩数组。网关检查时间戳、轨迹时域、位置/速度/加速度限制、端点步长和相对 JTC 当前参考的三次插值包络；不裁剪、重打时间戳或替换命令。

QP 诊断项名为 `elfin5_qp_controller`，状态取 `message` 冒号前的字段。可见状态有 `IDLE`、`WAITING_FOR_INPUT`、`RUNNING`、`REACHED`、`SINGULARITY_RECOVERY`、`SINGULARITY_STOP`、`QP_INFEASIBLE`、`QP_DEGRADED`、`INVALID_INPUT`、`STALE_INPUT`。网关仅接受 `RUNNING`、`REACHED`、`SINGULARITY_RECOVERY`；其余状态不满足放行条件。诊断键包括 `position_error_m`、`minimum_singular_value`、`qp_iterations`、`qp_projected_gradient_norm`。QP 状态与候选轨迹未按求解周期一一配对。

## 服务与网关状态机

| 服务 | 类型 | 契约 |
|---|---|---|
| `/elfin5_real/qp_candidate/enable` | `std_srvs/SetBool` | 开关候选计算；不使能电机，输出仍在隔离候选话题 |
| `/elfin5_real/bridge/enable` | `std_srvs/SetBool` | true 显式请求放行；false 关闭放行，原为 ACTIVE 时请求站点停止 |
| `/elfin5_real/bridge/reset` | `std_srvs/Trigger` | 人工清除网关锁存，回到 OBSERVE；不使能硬件、不恢复运动 |
| `/elfin5_site/stop` | `std_srvs/Trigger` | 站点实现、bridge 调用的停止契约，不是已核实的厂家服务别名 |
| `/controller_manager/list_controllers` | `controller_manager_msgs/ListControllers` | 厂家控制器管理器提供，bridge 只读查询 |

服务通信成功不等于业务成功，应检查响应 `success` 和 `message`。关闭网关服务的成功响应表示关闭/停止请求已受理，不代表停止服务已完成，更不代表机械臂已静止。

| 状态 | 含义与转换 |
|---|---|
| `OBSERVE` | 不下发轨迹；条件全部满足后自动进入 READY |
| `READY` | 可接受人工放行；条件失效回 OBSERVE，只有显式 enable(true) 才进入 ACTIVE |
| `ACTIVE` | 逐条检查并下发候选；人工关闭回 OBSERVE 并请求停止；控制必要条件失效进入 FAULT_LATCHED |
| `FAULT_LATCHED` | 禁止下发，保留首个控制故障原因并请求停止；数据恢复不会自动恢复运动 |

就绪条件包括：真实时钟、站点审查开关、设备标识与限值有效、六类输入有效且及时、JTC 正在运行并占用正确的六轴 `PositionJointInterface`、停止服务存在、控制器/发布者检查结果未过期。命令发布者应只有 bridge，候选发布者只有 qp_candidate，不能有竞争的 action goal 发布者。ROS 图检查是轮询快照，不是访问权限保障。

正常关闭后，停止请求结束且条件满足可再次 READY，无需 reset，但仍需人工重新 enable。停止调用失败会锁存故障并禁止 reset；处理现场原因后，仅显式再次调用 enable(false) 才重试停止。故障后的成功停止仍需人工确认实际停机、调用 reset，再重新检查条件并显式 enable。没有自动故障复位、自动伺服使能或自动恢复运动。

## RViz 与误差接口

以下话题前缀均为 `/elfin5_real/monitor/`：

| 后缀 | 类型 | 含义 |
|---|---|---|
| `current_point` | `geometry_msgs/PointStamped` | 真实关节反馈、模型和基座定位估计的 absolute_world TCP |
| `tracking_error` | `std_msgs/Float64` | 末端误差范数，m；无时间戳，分析优先使用 tracking_state |
| `tracking_state` | `diagnostic_msgs/DiagnosticArray` | `name=elfin5_real_tracking`；含 valid、position_error_m、base_stamp、target_stamp；无效时 WARN/NaN |
| `joint_tracking_error` | `sensor_msgs/JointState` | position 数组存六轴 `desired−actual`，rad，沿用同一 JTC state 的时间戳；不是机械臂关节角 |
| `path` | `nav_msgs/Path` | 有界长度的估计 TCP 位置历史，姿态为显示占位 |
| `markers` | `visualization_msgs/MarkerArray` | 目标、估计 TCP、误差连线及状态文字 |

末端误差为 `E(t)=||p_target_absolute_world(t)−p_tcp_absolute_world(t)||₂`，只画一条范数曲线；关节误差为六轴有符号过程跟踪误差。监视器以关节时间选择容许时间差内最近的基座样本，以及不晚于关节时间的最新目标，不插值伪造同步精度。末端误差是模型估计，不是外部测量的绝对精度。

真机 launch 参数：`hardware_config` 指定站点 YAML，`rviz` 默认 true，`base_tf_mode` 默认 pose。

```text
absolute_world（固定绝对系，不属于厂家 URDF）
  └─ 动态变换：monitor 或外部定位，二选一
      world（厂家模型根，随基座运动）
        └─ 厂家固定单位变换 world_base
            elfin_base_link → elfin_base → 六轴 → elfin_end_link
```

| 内容 | 唯一来源/使用者 |
|---|---|
| `/robot_description` | 厂家 bringup 加载完整 URDF；QP、bridge、monitor、RViz 共用 |
| `/robot_description_semantic`、`/robot_description_planning`、`/robot_description_kinematics` | QP launch 引用厂家 planning_context 加载；不启动 move_group，也不覆盖 URDF |
| `world → elfin_base_link` 及其他内部 TF | 厂家 `/robot_state_publisher`，读取真实 `/joint_states` |
| `absolute_world → world` | pose：monitor 根据 base_pose 发布；external：外部定位发布，monitor 不广播 |
| `/tf`、`/tf_static` | monitor 监听共享 TF；external 模式按定位时间查询 absolute_world 到 elfin_base_link 的完整链 |

monitor 初始化核验模型根到基座为固定单位变换，保证把 base_pose 赋给模型根在数学上成立。pose 模式只把其他发布者提供的 `world` 父变换判为根竞争，不再把厂家正常的 `world → elfin_base_link` 判为冲突。检测到竞争后停止自身根 TF 发布并将跟踪诊断置为无效，整理来源后重启监视器；这不增加网关的运动准入条件。

QP 和 bridge 没有 TF 查询，不需要新增 TF 订阅。QP 与 monitor 从共同模型计算相对基座的运动学，再结合 base_pose；共享 TF 用于显示及 external 模式核对，不能替代实际定位输入。RViz Fixed Frame 为 `absolute_world`，Robot Description 为 `/robot_description`。

`prepare_real_robot_description.py` 现在仅作为离线核验工具，成功后原样输出完整 XML；真机 launch 不调用它，不再删除 world 锚点。

## 参数与仿真接口

参数入口：[真机配置](../config/elfin5_real_robot.yaml)、[QP 配置](../config/elfin5_qp_controller.yaml)。参数在启动时读取。真实限值及尚待核实的时间阈值见[基本使用](usage.md)。

独立仿真入口 `elfin5_qp_gazebo.launch` 中，控制节点名为 `/elfin5_qp_controller`，直接使用 `/joint_states` 和 `/elfin_arm_controller/command`；私有 `base_pose`、`target_point`、`enable`、`state` 及 QP 观测话题位于该节点名下。该入口没有真机网关，须在与真机隔离的 ROS 环境中运行。
