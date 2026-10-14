# Elfin5 QP Controller V1 — ROS 接口

节点名：`/elfin5_qp_controller`

所有坐标长度单位为米，角度单位为弧度。`base_pose` 与 `target_point`
必须处在参数 `absolute_frame` 指定的绝对坐标系中；任一 `frame_id` 为空或
与该参数不一致时，控制器发布保持命令并拒绝运动。

当前接口只完成末端位置控制，不约束末端姿态，也不提供全局路径规划、环境避障、
夹爪控制或抓取执行。

## 输入话题

### `~base_pose`

- 类型：`geometry_msgs/PoseStamped`
- 发布方：未来由主控台发布；仿真阶段可由测试命令发布。
- 含义：参数 `base_link` 指定的机械臂基座坐标系在绝对坐标系中的位姿
  `T_absolute_base`；本版本默认对应URDF中的 `elfin_base_link`。
- 要求：四元数必须有效且非零。
- 超时：同时检查消息时间戳和本节点接收时间。

### `~target_point`

- 类型：`geometry_msgs/PointStamped`
- 发布方：未来由视觉或主控台发布。
- 含义：机械臂末端需要到达的绝对三维坐标。
- 说明：V1 不控制末端姿态，因此使用 `PointStamped` 而不是 `PoseStamped`。
- 超时：同时检查消息时间戳和本节点接收时间。

### `/joint_states`

- 类型：`sensor_msgs/JointState`
- 发布方：Gazebo 的 `joint_state_controller`。
- 必须包含：`elfin_joint1` 至 `elfin_joint6`。
- 要求：位置数组必须为有限数，并持续更新；过期后控制器进入保持状态。

## 输出话题

### `/elfin_arm_controller/command`

- 类型：`trajectory_msgs/JointTrajectory`
- 接收方：现有 `position_controllers/JointTrajectoryController`。
- 内容：按控制周期更新的单点短时域关节轨迹。

### `~current_point`

- 类型：`geometry_msgs/PointStamped`
- 内容：由正运动学计算的当前末端绝对坐标。
- 发布条件：`joint_states` 和 `base_pose` 有效即可；不要求控制器已经enable，也不
  要求已经收到target，便于在运动前选择安全目标点。

### `~position_error`

- 类型：`std_msgs/Float64`
- 单位：米。
- 内容：当前末端与目标点的欧氏距离。

### `~min_singular_value`

- 类型：`std_msgs/Float64`
- 内容：平移雅可比 `Jv` 的最小奇异值，用于判断位置任务奇异性。
- 发布条件：与 `~current_point` 相同，控制器未enable时也会发布。

### `~state`

- 类型：`diagnostic_msgs/DiagnosticArray`
- 状态：`IDLE`、`WAITING_FOR_INPUT`、`RUNNING`、`REACHED`、
  `SINGULARITY_RECOVERY`、`SINGULARITY_STOP`、`QP_INFEASIBLE`、
  `QP_DEGRADED`、`INVALID_INPUT`、`STALE_INPUT`。
- 关键字段：位置误差、最小奇异值、QP迭代次数和投影梯度残差。

## 服务

### `~enable`

- 类型：`std_srvs/SetBool`
- `data=true`：在 `joint_states`、`base_pose` 和 `target_point` 全部存在、未过期
  且坐标系正确时允许闭环运动；否则服务回复 `success: false` 并保持禁用。
- `data=false`：清零历史速度并发布一次当前位置保持命令，随后停止QP命令输出。
  这样既能中止当前短时域命令，也允许实验人员在QP关闭后用其他工具设置初始姿态。

## 命令行示例

以下示例匹配 `elfin_gazebo/launch/elfin5_empty_world.launch`：该文件使用
`-z 0.1` 生成模型，因此 `elfin_base_link` 的绝对高度填写为0.1米。目标点必须
按当前Elfin5工作空间调整。

```bash
rostopic pub -r 10 /elfin5_qp_controller/base_pose geometry_msgs/PoseStamped \
  "header: {frame_id: 'world'}
pose:
  position: {x: 0.0, y: 0.0, z: 0.1}
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}"

rostopic pub -r 10 /elfin5_qp_controller/target_point geometry_msgs/PointStamped \
  "header: {frame_id: 'world'}
point: {x: 0.35, y: 0.10, z: 0.45}"

rosservice call /elfin5_qp_controller/enable "data: true"
```

停止：

```bash
rosservice call /elfin5_qp_controller/enable "data: false"
```
