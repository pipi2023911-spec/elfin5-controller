# 测试结果

## 当前结论

本轮按开发要求只进行代码质量和交接文档整理，没有执行编译、单元测试或
Gazebo实验。因此目前没有可以登记为PASS的运行结果。

以下表格是后续接手者填写真实结果的唯一位置。没有命令输出、日志或数据文件时，
不得把状态从“未执行”改为“通过”。

## 测试环境

| 项目 | 当前记录 |
|---|---|
| 操作系统 | 未登记 |
| ROS版本 | 目标版本为ROS Melodic，未确认 |
| MoveIt版本 | 未登记 |
| Gazebo版本 | 目标版本为Gazebo 9，未确认 |
| Elfin5代码版本 | 未登记 |
| 本包提交版本 | 当前压缩包未包含Git提交信息 |

## 结果汇总

| 编号 | 测试项 | 验收条件 | 状态 | 证据路径 |
|---|---|---|---|---|
| T01 | 包编译 | 编译完成且无错误 | 未执行 | — |
| T02 | QP单元测试 | 全部测试通过 | 未执行 | — |
| T03 | Gazebo启动 | 模型、控制器和本节点正常启动 | 未执行 | — |
| T04 | 普通目标到达 | 最终位置误差不大于5 mm | 未执行 | — |
| T05 | base平移变换 | 绝对目标转换与独立计算一致 | 未执行 | — |
| T06 | base旋转变换 | 绝对目标转换与独立计算一致 | 未执行 | — |
| T07 | 近奇异位形恢复 | 无NaN、无越界并到达目标 | 未执行 | — |
| T08 | 指定奇异位形恢复 | 脱离奇异区并到达目标 | 未执行 | — |
| T09 | 输入超时 | 一个控制周期内进入保持状态 | 未执行 | — |
| T10 | 关节约束 | 位置、速度和加速度零违规 | 未执行 | — |
| T11 | QP降级状态 | 未收敛可行解被标记为QP_DEGRADED | 未执行 | — |
| T12 | 禁用态被动观察 | 无目标时持续输出current_point和sigma，机械臂不动 | 未执行 | — |
| T13 | 指令源切换 | MoveIt设置初始姿态与QP运动不同时发令 | 未执行 | — |
| T14 | enable输入门禁 | 输入缺失、过期或frame错误时服务拒绝启用 | 未执行 | — |

## 实验前门禁

必须按顺序完成以下门禁。前一项不满足时，不进入后一项，也不通过增大速度或放宽
奇异阈值来绕过问题。

### G0：确认使用的代码版本

1. 确认catkin工作空间中只有一个 `elfin5_qp_controller`；
2. 确认配置中存在：

```yaml
base_link: elfin_base_link
absolute_frame: world
qp_max_iterations: 100
qp_tolerance: 1.0e-7
```

3. 确认 `docs/DEV_LOG.md` 包含“匹配 elfin_robot-melodic 配置包”记录；
4. 记录当前Git提交；如果没有Git提交，先提交后再实验，避免结果无法对应代码。

预期结果：能够唯一确定正在运行的是本次修订后的包。

### G1：只检查构建和参数加载

在catkin工作空间中执行：

```bash
catkin_make --pkg elfin5_qp_controller
source devel/setup.bash
rospack find elfin5_qp_controller
rospack find elfin_gazebo
rospack find elfin5_moveit_config
```

预期结果：编译无错误，三个包都解析到当前工作空间预期目录。

若失败：先解决依赖、重复工作空间或旧版overlay问题，不启动Gazebo。

### G2：启动底层Gazebo，不启动QP运动

建议首次调试分三个终端启动，便于判断故障属于哪一层：

```bash
# 终端1
roslaunch elfin_gazebo elfin5_empty_world.launch

# 终端2
roslaunch elfin5_moveit_config moveit_planning_execution.launch display:=false

# 终端3
roslaunch elfin5_qp_controller elfin5_qp_controller.launch
```

确认底层控制器和话题：

```bash
rosservice call /controller_manager/list_controllers
rostopic info /elfin_arm_controller/command
rostopic hz /joint_states
rosparam get /robot_description_semantic
rosparam get /elfin5_qp_controller/base_link
```

预期结果：

- `joint_state_controller` 和 `elfin_arm_controller` 为running；
- `/elfin_arm_controller/command` 至少有一个订阅者；
- `/joint_states` 接近50 Hz；
- `robot_description_semantic` 存在；
- `base_link` 为 `elfin_base_link`。

### G3：发布base，保持enable=false进行被动观察

该Gazebo启动文件以 `-z 0.1` 生成模型，因此发布：

```bash
rostopic pub -r 10 /elfin5_qp_controller/base_pose geometry_msgs/PoseStamped \
  "header: {frame_id: 'world'}
pose:
  position: {x: 0.0, y: 0.0, z: 0.1}
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}"
```

保持控制器禁用，并观察：

```bash
rostopic echo /elfin5_qp_controller/current_point
rostopic echo /elfin5_qp_controller/min_singular_value
rostopic echo /elfin5_qp_controller/state
```

预期结果：

- `current_point` 和 `min_singular_value` 持续更新；
- 状态为 `IDLE`；
- 机械臂保持不动；
- 所有数值有限，没有NaN或Inf。

若当前关节零位的 `min_singular_value` 很小，这是待测对象，不在本门禁阶段启用运动。

### G3.1：确认运动指令只有一个来源

`/elfin_arm_controller/command` 不能同时被QP节点、MoveIt执行器或手工
`rostopic pub` 用来驱动机械臂。首次运动前执行：

```bash
rostopic info /elfin_arm_controller/command
rostopic info /elfin_arm_controller/follow_joint_trajectory/goal
```

执行原则：

- 需要用MoveIt设置初始关节姿态时，先保持 `enable=false`；
- 禁用QP后会发布一次保持命令，随后QP节点停止发布运动命令；
- 等MoveIt动作完成后，不再发送新的MoveIt目标，再重新读取
  `current_point` 并设置QP目标；
- QP运动期间不在RViz中点击Execute，不发布手工关节轨迹。

如果无法明确当前运动指令的唯一来源，不进入G4。

### G4：选择第一个普通目标

从 `current_point` 读取当前绝对坐标，只在一个轴增加1～2厘米，并确认该方向不会
立即接近地面或机械臂本体。例如：

```text
target_x = current_x + 0.01
target_y = current_y
target_z = current_z
```

不要把README中的示例点直接当作首次运动目标；示例只说明消息格式，不保证它离
当前末端足够近。

以10 Hz持续发布目标，确认状态仍为 `IDLE` 后再enable：

```bash
rostopic pub -r 10 /elfin5_qp_controller/target_point geometry_msgs/PointStamped \
  "header: {frame_id: 'world'}
point: {x: TARGET_X, y: TARGET_Y, z: TARGET_Z}"

rosservice call /elfin5_qp_controller/enable "data: true"
```

服务必须返回 `success: true`。如果返回 `success: false`，按回复中的原因修复输入，
不要通过反复调用enable绕过门禁。

执行enable之前必须再确认：

- base和target发布端均在持续运行，频率高于 `1 / input_timeout`；
- `current_point`、target和Gazebo中预期运动方向一致；
- 没有正在执行的MoveIt目标或手工轨迹发布器；
- 另一个终端已准备好disable命令，Gazebo界面可随时暂停。

### G4.1：停止与异常处置顺序

正常结束或观察到方向错误、抖动、关节突跳时，按以下顺序处理：

```bash
# 1. 首先停止QP生成新命令
rosservice call /elfin5_qp_controller/enable "data: false"

# 2. 若服务无响应或运动仍在继续，暂停Gazebo物理
rosservice call /gazebo/pause_physics
```

暂停后先保存日志和rosbag，再停止节点。不先删除日志、不立即修改多个参数。
处理完成后如需继续，先执行：

```bash
rosservice call /gazebo/unpause_physics
```

然后从G3重新确认被动观察数据，不直接重新enable。

### G5：普通小步运动通过后再逐级增加难度

固定顺序：

1. 同一base位姿下，三个1～2厘米普通目标；
2. 同一base位姿下，三个较远但可达目标；
3. 修改base平移，检查同一绝对目标的转换；
4. 修改base绕Z轴旋转，检查坐标轴方向；
5. 手动设置近奇异但非精确奇异的初始关节角；
6. 最后测试项目指定的精确奇异位形。

任何一级失败，都回到该级调试，不继续增加难度。

设置第5、6级初始姿态时，必须重复G3.1的指令源切换流程：先禁用QP，
再用MoveIt设置姿态，动作完成后停止MoveIt执行，最后更新目标并启用QP。

## 实验期间必须同时记录的话题

建议使用rosbag统一记录：

```bash
rosbag record -O elfin5_qp_run.bag \
  /joint_states \
  /elfin_arm_controller/command \
  /elfin5_qp_controller/base_pose \
  /elfin5_qp_controller/target_point \
  /elfin5_qp_controller/current_point \
  /elfin5_qp_controller/position_error \
  /elfin5_qp_controller/min_singular_value \
  /elfin5_qp_controller/state
```

每次实验前开始记录，实验结束或失败后立即停止并填写实验编号。

## 故障分流

| 现象或状态 | 优先检查 | 不应立即做的事 |
|---|---|---|
| 节点启动即退出 | `robot_description`、SRDF、group/base/tip名称 | 修改QP参数 |
| `WAITING_FOR_INPUT` | 缺少哪个输入话题、消息频率 | enable/disable反复重试 |
| `INVALID_INPUT` | frame_id、NaN、关节名称和四元数 | 放宽奇异阈值 |
| `STALE_INPUT` | 三个输入的发布频率、时间戳和ROS时间 | 增大速度 |
| 有command但机械臂不动 | controller是否running、command订阅者、命令内容 | 修改Jacobian |
| 机械臂突然改变方向 | 是否存在MoveIt、QP或手工轨迹竞争 | 先调整增益 |
| 末端方向旋转90° | `base_link`、base_pose语义、当前包是否为修订版 | 调Kp补偿方向错误 |
| `QP_INFEASIBLE` | 关节是否接近限位、上下界、奇异半空间 | 直接取消全部安全约束 |
| 持续 `QP_DEGRADED` | 迭代次数、投影梯度残差、控制周期耗时 | 把状态当作完全收敛 |
| `SINGULARITY_STOP` | 保存初始关节角、sigma和恢复梯度相关日志 | 随机改变多个参数 |
| 速度抖动 | command时域、目标跳变、Kp、QP状态 | 一次同时改多个参数 |
| 到点后仍缓慢移动 | goal tolerance、joint feedback和保持命令 | 继续扩大目标距离 |

## 参数调整纪律

- 首轮普通点实验不修改默认参数；
- 每次只修改一个参数，并在实验记录中保存修改前后值；
- 先确认坐标系和输入正确，再调整 `position_gain`；
- 先记录正常姿态的 `min_singular_value`，再调整奇异阈值；
- `qp_max_iterations` 和 `qp_tolerance` 只根据QP迭代次数、残差和控制周期耗时调整；
- 不允许通过关闭位置、速度或加速度边界来获得“通过”结果。

## Gazebo实验记录模板

每组实验至少填写以下字段：

```text
实验编号：
日期：
代码提交：
初始关节角：
base绝对位姿：
目标绝对坐标：
最终位置误差：
最小sigma_min：
最大关节速度：
QP最大迭代次数：
最终控制器状态：
PASS/FAIL：
日志路径：
数据文件路径：
异常与处理：
```

## 证据要求

Gazebo阶段至少保留：

- 完整启动日志；
- `position_error` 和 `min_singular_value` 的时间序列；
- 六个关节的位置与速度；
- 控制器状态、QP迭代次数和QP残差；
- 指定奇异初始姿态的关节角；
- 一段能够看到初始姿态、运动过程和最终位置的录屏。
