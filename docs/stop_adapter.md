# stop适配器与Windows验证

## 当前完成等级

当前开发环境为Windows。本轮编译并运行实际C++适配器，ROS类型、话题、订阅连接、参数和服务注册使用桩；23个场景、76项断言通过。**这不是完整ROS构建、Gazebo执行或真机停车验证。** 没有启动厂家驱动、使能机械臂或改变网关放行条件。

`src/stop_hold_adapter_node.cpp`提供一个独立的受控停止请求和反馈确认接口。真机主launch没有自动启动它，节点构造不发任何轨迹；只有明确调用服务时才发送一条空`JointTrajectory`。依据是已审核的Melodic JTC空轨迹保持路径，仍需匹配现场安装版本及厂家`stop_trajectory_duration`配置。

## 放行依赖的stop

网关`enable(true)`只检查停止服务存在，不会在放行时调用它。故障时先阻断普通候选并锁存，再由停止线程调用该服务。适配器服务存在、进程启动或返回成功，都不替代现场停止链核验；`site_review_complete=false`和站点空限值数组保持不变。

服务成功的准确含义是：已发出停止请求，并观察到**请求后的六轴实际速度与JTC期望速度，在确认窗口内持续低于阈值**。它不提供原生命令接收ACK，不证明没有其他发送源随后覆盖停止，也不是安全等级急停认证。调用前应由网关阻断新候选、停止其他轨迹/action发送源；通信或驱动失效时，依赖独立执行层停止措施。

## 接口与参数

| 项目 | 默认值 | 说明 |
|---|---|---|
| stop_service | `/elfin5_site/stop_hold` | `std_srvs/Trigger`，并发请求返回失败，不重复发送 |
| command_topic | `/elfin_arm_controller/command` | 仅在服务调用时发送空轨迹，不发送运动目标 |
| joint_states_topic | `/joint_states` | 按六轴名称读取实际velocity |
| controller_state_topic | `/elfin_arm_controller/state` | 按名称读取desired.velocities |
| joint_names | elfin_joint1至elfin_joint6 | 六个不同非空名称，可通过节点私有参数配置 |
| still_velocity | 0.005 rad/s | 六轴绝对速度必须严格小于阈值，待现场核验 |
| confirm_duration | 0.2 s | 两路都须有跨越该时长的新样本，不能等待单帧零速变成“持续” |
| confirm_timeout | 1.0 s | 使用`steady_clock`的确认期限，不受ROS时间冻结影响；不是硬实时停车时限 |
| feedback_timeout | 0.3 s | 同时检查接收年龄与源时间年龄，待现场核验 |

消息必须含全部六轴，name与velocity长度一致、名称不重复、所需速度有限。额外工具关节可以存在，但不代替机械臂六轴。源时间戳须非零、严格前进、晚于当前请求；未来容差固定为10 ms，不能用未来或回放数据确认停车。任一路无效、运动或超时都会重置低速确认；两路均须在窗口末端实际收到新样本。

没有command订阅者、发布异常、发布后断连、ROS时间回退、并发调用或确认超时都返回`success=false`。正响应也不能证明真实JTC已经执行命令：本节点只能以当前接口发送请求并观测状态，现场仍需验证命令通路、控制器运行、停止位移及故障处理。

`confirm_timeout`约束本节点的确认循环，不是整个ROS服务传输的硬超时，也不是机械停车时限。ROS消息继续刷新时间戳不等于PDO数据仍新鲜，不能靠本节点替代总线失效保护。

## Windows已执行测试

使用已有Python与Zig编译器，从仓库根目录：

```powershell
python test/test_stop_hold_adapter.py --zig ../build-tools/ziglang/zig.exe
python test/test_real_robot_tf_and_error.py -v
```

适配器测试编译整个生产源文件，只公开成员并把生产main重命名；构造函数、回调和服务方法实际运行，通信均为桩。覆盖启动无输出、新鲜/乱序名称成功、生产默认参数、旧零速、单帧新零速、缺轴、长度不符、NaN、重复名称、真实或期望仍运动、刚好达到阈值、旧/未来/零时间戳、无JTC状态、无连接、发布异常、冻结/回退ROS时间、并发请求、短暂低速及发布后断连。测试结果是协议行为，不能写成机械臂实验成功。

首轮夹具把新鲜度窗口设为15 ms，在本机fresh案例失败；后改用120 ms窗口和40 ms确认时间，避免把Windows线程调度当作实时保证。生产0.3/0.2/1.0秒参数没有因此改变，并另有生产默认值的正向测试。首次失败日志作为Debug材料保留。

## 后续ROS/Gazebo与现场步骤（未执行）

Ubuntu ROS Melodic完整构建后，可在隔离环境单独启动：

```bash
roslaunch elfin5_qp_controller elfin5_stop_hold_adapter.launch
```

话题和服务均可用launch参数改到隔离命名空间。必须先用隔离master/Gazebo验证消息连接、并发反馈、服务等待、断连/停发/回退以及网关故障后的联动。服务等待使用两个spinner线程，桩无法证明真实ROS调度/ABI正确。CMake已添加节点和离线测试注册，但本轮未在ROS下运行。

现场停止验证须单独获得原有现场授权：匹配JTC版本及停止配置，限制低速、小范围和实际负载，记录请求、desired/actual速度、时间戳、停止位移及反馈/驱动异常。不得仅因为Windows测试通过就设置现场审查标志或放行。完整证据与失败记录见[验证报告](evidence/stop-horizon-windows-2026-10-25/README.md)。
