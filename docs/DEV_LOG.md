# 开发日志

本文件记录代码修改事实、修改原因和仍未验证的风险。运行结果统一记录在
`TEST_RESULTS.md`，避免把开发判断与实验事实混在一起。

## 2026-09-22：安全到点控制器代码质量整理

### 本次范围

- 保留位置级笛卡尔控制目标；
- 不增加末端姿态控制；
- 不增加MoveIt全局路径规划、碰撞检查、夹爪或抓取状态机；
- 本轮按要求不执行编译、单元测试和Gazebo实验。

### 输入与安全处理

- 在三个ROS输入回调中增加有限数检查；
- 为 `joint_states`、`base_pose` 和 `target_point` 分别记录本节点接收时间；
- 控制周期同时检查消息时间戳和接收时间；
- 将 `/joint_states` 纳入超时保护；
- 要求 `base_pose` 和 `target_point` 明确使用配置的 `absolute_frame`；
- 将关节名称重排提取为独立函数，控制主流程不再混入数组查找细节；
- 所有可获得当前关节位置的异常分支都会发布零速度保持命令。

### 并发状态处理

- `data_mutex_` 只保护ROS输入缓存；
- 新增 `control_mutex_`，串行化MoveIt `RobotState`、历史速度和QP控制周期；
- 重叠控制周期会被跳过并输出节流警告；
- enable服务与控制循环采用相同锁顺序，避免数据竞争和死锁。

### 奇异性处理

- 保留3×N平移雅可比最小奇异值作为位置任务指标；
- 常规区域继续使用中央差分梯度；
- 当中央差分在精确奇异点附近退化为零时，比较每个关节正负单侧扰动；
- 只把确实能够提高最小奇异值的扰动写入恢复方向；
- 扰动步长会受关节位置边界限制，不再无条件跨越模型关节边界。

### 轨迹命令

- 区分控制周期 `control_dt` 和命令时域 `command_horizon`；
- 加速度限制使用实际控制周期；
- 位置预测限制、位置积分和 `time_from_start` 使用同一个命令时域；
- 修复原代码中 `q + qdot * dt` 与 `time_from_start = factor * dt` 不一致的问题。

### QP求解状态

- 增加矩阵、向量、半空间维度和有限数检查；
- 增加求解器配置检查；
- `success` 只表示结果数值有效且满足安全约束；
- 新增 `converged` 表示投影梯度是否达到容差；
- 可行但未收敛的结果发布为 `QP_DEGRADED`，不再报告为完整求解成功；
- 诊断消息新增QP迭代次数和投影梯度残差。

### 尚未验证

- 修改后的代码尚未在ROS Melodic环境编译；
- 单侧恢复方向尚未通过Elfin5实际奇异位形数据验证；
- 自制投影梯度求解器的运行时间和收敛质量尚无Gazebo数据；
- 底层 `JointTrajectoryController` 对高频单点轨迹的实际插值表现尚未确认。

## 2026-09-22：匹配 elfin_robot-melodic 配置包

### 接口核对结果

- 规划组 `elfin_arm` 与 `elfin5.srdf` 一致；
- 末端链接 `elfin_end_link` 与URDF、SRDF一致；
- 六个关节名称、速度限制和加速度限制与MoveIt配置一致；
- `/elfin_arm_controller/command` 与Gazebo中的
  `position_controllers/JointTrajectoryController` 一致；
- `/joint_states` 与Gazebo的 `joint_state_controller` 一致。

### 修复的坐标接口错误

Elfin5 URDF在 `elfin_base_link` 与 `elfin_base` 之间定义了绕Z轴90度的固定旋转。
MoveIt `getJacobian()` 把Jacobian表达在规划组首关节父链接 `elfin_base` 中，旧代码
却直接把模型 `world` 坐标中的末端位置当作base坐标，导致位置误差和Jacobian
不在同一个坐标系中。

本次新增 `base_link` 参数，默认值为 `elfin_base_link`。末端FK显式转换到该链接，
平移Jacobian也从 `elfin_base` 旋转到同一坐标系后再进入QP。

### 其他接口修正

- 完整启动文件依赖 `elfin_gazebo` 和 `elfin5_moveit_config`，已在
  `package.xml` 中补充运行依赖；
- Eigen依赖使用ROS包名 `eigen`；
- 上游Gazebo通过 `-z 0.1` 生成模型，README和接口示例中的基座高度同步改为
  0.1米。

### 未修改的上游文件

用户提供的 `elfin_robot-melodic` 原始目录只用于接口核对，本次没有修改其中的
URDF、MoveIt配置、Gazebo配置或控制器代码。

## 2026-09-22：实验前可观测性复核

### 发现的问题

旧控制流程在 `enable=false` 时直接返回，因此首次实验无法在机械臂保持状态下读取
`current_point` 和 `min_singular_value`。实验人员只能猜测目标位置后再启用控制，
不利于安全调试。

### 代码调整

- 将base输入检查和运动学计算移动到enable判断之前；
- 只要 `joint_states` 与 `base_pose` 有效，即使控制器未启用，也持续发布
  `current_point` 和 `min_singular_value`；
- target只在enable之后才成为必需输入；
- 将QP最大迭代次数和收敛容差改为YAML参数；
- disable时只发布一次保持命令，之后不持续占用command话题，便于设置实验初始
  姿态；其他异常分支仍持续发布保持命令。

### 调试流程调整

首次实验固定采用以下顺序：接口检查、被动观察、普通小步运动、坐标变换、近奇异、
指定奇异。不得从零位奇异恢复直接开始测试。

### 实验前二次复核

- 新增G0至G5逐级门禁，前一级失败时不进入后一级；
- enable服务在输入缺失、过期、关节名不完整或frame错误时直接拒绝运动；
- 新增运动指令唯一来源规则，MoveIt只在QP禁用时设置初始姿态；
- 新增disable与Gazebo暂停的异常处置顺序；
- 完整启动文件删除未传给上游的无效 `paused` 参数，避免产生错误预期；
- 增加禁用态被动观察和指令源切换两项待执行验收项。
