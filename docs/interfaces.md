# 接口说明

## 控制对象、坐标与单位

仅控制末端 XYZ，不主动约束姿态。位置m、角度rad、速度rad/s、加速度rad/s²、时间s。配置数组按 `joint_names` 排列，消息按名称重排。

`absolute_world` 是固定绝对系；`base_pose` 是 `elfin_base_link` 在该系的位姿，`target_point` 是同系目标。厂家模型根 `world` 与 `elfin_base_link` 必须固定重合。模型内部变换直接读取URDF，不能假定另有90度偏转。

## 节点与数据流

```text
/joint_states、/base_pose、/target_point → qp_candidate → candidate_trajectory
/joint_states、JTC state、厂家enable/fault → bridge → /elfin_arm_controller/command
同一组实际反馈 → monitor → TF、TCP、误差、可视化
```

QP自己检查算法输入。网关不转发validated输入、不订阅QP诊断或基座/目标、不扫描ROS图。人工放行时查询一次controller_manager，核对控制器running、类型、PositionJointInterface和六轴资源。运行中监督反馈及厂家状态；不会重新检查ROS图中的发布者独占性。现场应停止其他轨迹/action发送源，不能将节点名称或话题当成访问控制。

## QP接口

以真机命名空间 `/elfin5_real/qp_candidate` 为准；Gazebo为 `/elfin5_qp_controller`。

| 接口 | 类型 | 含义 |
|---|---|---|
| 输入 `/joint_states` | sensor_msgs/JointState | 六轴实际关节位置 |
| 输入 `/base_pose` | geometry_msgs/PoseStamped | 真机基座绝对位姿 |
| 输入 `/target_point` | geometry_msgs/PointStamped | 真机绝对目标XYZ |
| `~/enable` | std_srvs/SetBool | 仅启用/关闭QP计算，不使能驱动，不放行网关 |
| `~/current_point` | geometry_msgs/PointStamped | FK当前末端位置，时间为计算时刻 |
| `~/position_error` | std_msgs/Float64 | XYZ误差范数，m，无源时间戳 |
| `~/min_singular_value` | std_msgs/Float64 | 平移Jacobian最小奇异值 |
| `~/state` | diagnostic_msgs/DiagnosticArray | 状态、误差、奇异值、QP迭代次数/残差 |
| 真机候选 `/elfin5_real/candidate_trajectory` | trajectory_msgs/JointTrajectory | 由launch将QP绝对command输出重映射到此处 |
| Gazebo输出 `/elfin_arm_controller/command` | trajectory_msgs/JointTrajectory | Gazebo入口直接发送到仿真JTC |

QP的base/target需要持续更新且坐标系正确；不能只改frame_id冒充坐标变换。状态包括IDLE、WAITING_FOR_INPUT、INVALID_INPUT、STALE_INPUT、RUNNING、REACHED、QP_DEGRADED、QP_INFEASIBLE和奇异性相关状态。诊断用于分析，不作为网关逐候选授权凭证。

## 网关输入输出

| 默认话题 | 类型 | 用途 |
|---|---|---|
| `/joint_states` | sensor_msgs/JointState | 通过名称、有限值、位置限位和时间检查后缓存实际姿态 |
| `/elfin_arm_controller/state` | control_msgs/JointTrajectoryControllerState | 同一消息的desired位置/速度作为插值参考，检查actual位置 |
| `/elfin_ros_control/elfin/enable_state` | std_msgs/Bool | 厂家使能聚合，true为已使能 |
| `/elfin_ros_control/elfin/fault_state` | std_msgs/Bool | 厂家故障聚合，true为有故障 |
| `/elfin5_real/candidate_trajectory` | trajectory_msgs/JointTrajectory | QP候选输入 |
| `/elfin_arm_controller/command` | trajectory_msgs/JointTrajectory | 实际输出，不锁存，队列1；包括获准候选及一次故障保持请求 |
| `/elfin5_real/bridge/state` | std_msgs/String | 英文状态码前缀＋中文原因 |

Bool只有本机单调接收时间。关节/JTC同时检查源时间与单调接收年龄，并检查两种反馈源时间差。新JointState并不能证明其底层PDO缓存是新的；EtherCAT故障/恢复还依赖现场已验证的链路监测与看门狗。

### 候选契约

单个轨迹点，六轴位置与速度；不接受额外加速度/effort字段。要求有限值、正确名称、生成时间晚于本次放行和上一已接收命令、年龄不超限、剩余执行时域有效。

按反馈和JTC参考检查端点位置/偏移，并保留三次Hermite/Bernstein的位置、速度、加速度包络检查。这是基于异步观测参考的保守筛查，不能证明厂家JTC实际拼接轨迹完全相同。网关不修改通过的候选内容、不重新积分、不重打时间戳。

### 人工服务

| 服务 | 类型 | 语义 |
|---|---|---|
| `/elfin5_real/bridge/enable` | std_srvs/SetBool | true人工放行；false关闭、请求保持/停止并锁存；停止失败后再次false明确重试一次 |
| `/elfin5_real/bridge/reset` | std_srvs/Trigger | 停止请求成功应答后，操作员确认停稳再清锁存；不恢复运动 |
| `/controller_manager/list_controllers` | controller_manager_msgs/ListControllers | 人工放行时查询执行控制器 |
| `/elfin5_site/stop_hold` | std_srvs/Trigger | 由站点提供，经验证的停止当前运动并保持；本包只有客户端 |

服务通信成功不等于业务成功，应检查 `success` 和中文 `message`。enable(false)成功表示网关处理了关闭请求，不表示已物理停稳。

### 状态与故障保持

内部只有OBSERVE、ACTIVE、FAULT_LATCHED；READY是OBSERVE下就绪条件成立的显示值，不会自动进入ACTIVE。

| 触发情况 | 网关动作 |
|---|---|
| 启动/未人工放行 | 候选丢弃，不生成保持输出 |
| 人工放行且检查通过 | 原样转发新候选 |
| 非法候选、候选超时、人工关闭；实际反馈/驱动仍有效 | 立即锁存，尝试一次已验证插值边界内的保持轨迹，同时请求站点停止保持 |
| 关节/JTC反馈失效或超时、驱动故障/失能、时间回退 | 锁存并请求站点停止保持；不凭过期/无效位置发回拉轨迹 |
| 保持轨迹超限或不可构造 | 不发送该保持轨迹，仍请求站点停止保持 |
| 恢复反馈/驱动正常 | 保持锁存，不自动转发 |
| 站点服务失败 | 保持锁存、报告中文失败；等待现场独立处置和人工重试 |

“最近有效姿态”指最近通过名称、有限值、限位与时间检查的实际关节反馈，不是最近QP目标，不是历史任意一个姿态，也不证明碰撞安全。保持轨迹目标是该姿态，端点速度为零，使用正执行时域；它不是瞬时强制零速。通过包络才会发送，目标只发送一次，不追逐漂移的反馈。

任何触发都不能凭网关消息保证停稳。反馈超时或驱动故障时，旧位置可能不可达或引起回退，此时由经过验证的执行层停止保持路径处理。不存在“不依赖反馈和驱动状态也能保证旧姿态静止”的软件承诺。

### 站点stop_hold契约

实现应停止/替换当前运动，使用可靠执行层信息实现受控保持，避免继续执行旧候选；失败明确回复false。必须核实它对正常运动、反馈中断、总线异常和驱动故障的实际行为，记录响应和关节速度。它不能是空服务、固定成功回执，不能直接把Trigger重映射到SetBool。

厂家 `stop_teleop` 发送空action轨迹、`disable_robot` 撤销使能，均不自动等价于受控停止保持。本包不自动调用使能、clear_fault或解除抱闸。保持失败时依赖现场独立急停；软件进程退出、网络中断或停止服务卡住仍需要执行层看门狗。

## 网关参数

| 参数 | 含义 |
|---|---|
| joint_names | 六个唯一名称，数组顺序 |
| position_lower / position_upper | 现场位置上下限，不得超出URDF |
| velocity_limits / acceleration_limits | 现场运动限制；速度不得超出URDF |
| max_endpoint_step | 候选端点相对实际位置和JTC参考的最大变化 |
| min_horizon / max_horizon | 允许轨迹时域范围 |
| hold_horizon | 保持请求时域，必须在上述范围内，默认0.10s；不是已测停稳时间 |
| input_timeout | 关节/JTC源时间和接收年龄、Bool接收年龄限值 |
| max_input_skew | 关节与JTC源时间差上限 |
| candidate_max_age / candidate_timeout | 单条命令年龄 / 持续无合格候选的超时 |
| site_review_complete | 人工已完成限值、插值、链路和停止保持审查的记录，默认false；标志本身不是验证 |
| *_topic / *_service | 输入输出及厂家/站点接口，可改名；候选与输出不得相同 |
| controller_name / controller_type | 人工放行时要求的执行控制器 |

时间阈值必须依据实测调整，不能为了通过检查随意放宽。参数启动时读取，不热更新。

## 监视与数据接口

monitor默认发布 `~/current_point`、`~/tracking_error`、`~/tracking_state`、`~/joint_tracking_error`、`~/markers`、`~/path`。误差来自实际反馈FK/定位估计，不是外部测量TCP精度。

`base_tf_mode=pose` 时monitor发布 `absolute_world→world`；`external` 时核对外部动态变换。厂家robot_state_publisher独占模型内部TF，两者不能重复发布同一根变换。monitor退出或绘图失败不会给网关增加放行条件。

JTC `desired−actual` 是过程跟踪误差；未来候选端点不能直接减当前反馈作为同一时刻的跟踪误差。记录时保留源时间、接收时间、参考/反馈及候选时域。
