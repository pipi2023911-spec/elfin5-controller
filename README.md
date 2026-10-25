# Elfin5 XYZ QP 控制器

ROS1 机械臂实验包：通过 QP 计算六轴关节速度，跟踪末端 XYZ 位置，输出单点短时域关节轨迹。姿态允许变化，不主动控制姿态。

支持 Gazebo 实验、读取真机反馈的影子计算，以及默认关闭的真机薄网关。代码存在、编译通过、仿真通过和真机闭环验证属于不同完成等级。

## 控制链与职责

```text
关节反馈 + 基座位姿 + 目标XYZ → QP → 候选轨迹
                                      ↓
                         网关：人工放行、命令检查
                                      ↓
                  厂家位置型JTC → RobotHW → EtherCAT
```

网关不代理 QP 输入、不扫描 ROS 图、不解析 QP 诊断文本、不自动使能或清除厂家故障。监视器与 RViz 只负责显示和记录。

故障时先阻断候选并锁存：反馈和设备仍有效时，请求保持最近通过限位检查的实际反馈姿态，保持端点速度为零；反馈失效、驱动故障或保持轨迹检查失败时，不向旧姿态回拉，调用现场已验证的停止保持服务。保持请求和服务成功回执都不能单独证明机械臂已经静止。详见[接口与故障保持](docs/interfaces.md)。

## 环境与入口

目标环境：Ubuntu 18.04、ROS Melodic、Python 2.7、C++14、Eigen、MoveIt。厂家包应与设备及现场标定一致。厂家驱动中的数组可能按 J2/J1 排列，本包按关节名称映射，不能直接按索引混用。

当前开发机是Windows：本轮使用Python3和Zig执行离线逻辑/时序验证，未使用ROS、Gazebo或真机执行环境。目标部署环境与已验证环境分别记录。

- [操作与编译](docs/usage.md)：环境准备、仿真、影子计算、人工放行、关闭/复位和数据分析。
- [接口说明](docs/interfaces.md)：话题、服务、参数、坐标、时间与故障行为。
- [测试说明](docs/testing.md)：独立 C++、Python 离线、ROS 隔离、Gazebo 和现场验收。
- [QP算法与验收](docs/qp_solver.md)：有效集、半空间乘子搜索、残差定义与数值回归。
- [stop适配器](docs/stop_adapter.md)：独立停止请求、请求后持续低速确认和Windows验证边界。
- [时域与简化审查](docs/horizon_validation.md)：50/140 ms离线对照、回调延迟、停发代价及未实施项。

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
roslaunch elfin5_qp_controller elfin5_qp_gazebo.launch
```

Gazebo 启动后还需提供基座位姿、目标并显式启用 QP；完整步骤在操作文档。真机入口不启动 EtherCAT 或厂家控制器。

## 全部文件说明

| 文件 | 职责 |
|---|---|
| `README.md` | 项目范围、文件导航和验证边界 |
| `CMakeLists.txt` | 构建库/节点、安装脚本/文档、注册测试 |
| `package.xml` | ROS 编译、运行和测试依赖 |
| `.gitignore` | 排除构建、录包、日志和本机缓存 |
| `.gitattributes` | Git 文本文件属性 |
| `include/elfin5_qp_controller/box_qp_solver.h` | QP 解状态与求解器接口 |
| `include/elfin5_qp_controller/qp_cartesian_controller.h` | XYZ 控制器成员、回调与运动学接口 |
| `include/elfin5_qp_controller/real_robot_command_gate.h` | 纯 C++ 限值、候选校验、保持命令接口 |
| `src_gazebo/box_qp_solver.cpp` | 盒有效集与半空间乘子搜索，完整KKT证书判定 |
| `src_gazebo/qp_cartesian_controller.cpp` | FK/Jacobian、XYZ误差、QP组装、约束、奇异规避、候选发布 |
| `src_gazebo/qp_cartesian_controller_node.cpp` | QP ROS 入口与回调线程 |
| `src/real_robot_command_gate.cpp` | 名称映射、时间/限位/插值包络检查及零端点速度保持构造 |
| `src/real_robot_bridge_node.cpp` | 人工输出开关、厂家状态监督、故障锁存、一次保持与异步停止请求 |
| `src/real_robot_monitor_node.cpp` | 基座与关节反馈的FK、TF责任检查、TCP/关节误差与可视化 |
| `src/stop_hold_adapter_node.cpp` | 可选站点停止请求/反馈确认节点；不使能驱动，未完成现场停车验证 |
| `launch/elfin5_qp_controller.launch` | 单独加载 QP，依赖外部模型和执行环境 |
| `launch/elfin5_qp_gazebo.launch` | 厂家 Gazebo、MoveIt 与 QP 入口，不加载真机网关 |
| `launch/elfin5_qp_real_robot.launch` | 真机候选 QP、网关、监视器与可选 RViz |
| `launch/elfin5_stop_hold_adapter.launch` | 独立启动适配器，可改为隔离话题；真机主入口不自动包含 |
| `config/elfin5_qp_controller.yaml` | QP参数、控制频率、关节约束和奇异阈值 |
| `config/elfin5_real_robot.yaml` | 真机限值、厂家状态/停止接口、监视与低速候选参数 |
| `config/elfin5_real_robot.rviz` | absolute_world 下的模型、TF、轨迹和目标显示 |
| `config/qp_test_suite.yaml` | Gazebo轮次、初始姿态、目标和判定阈值 |
| `scripts/run_qp_test_suite.py` | Gazebo多轮实验、录包、CSV与结果汇总；目标Python2.7 |
| `scripts/prepare_real_robot_description.py` | 离线核对完整URDF并原样输出，不修改厂家模型 |
| `scripts/plot_real_robot_error.py` | 分析rosbag，输出TCP误差、关节参考/反馈/误差、统计与事件图 |
| `scripts/gen_qp_reference_cases.py` | 开发机Python3/NumPy/SciPy参考集生成，保存种子、版本与拒收次数 |
| `scripts/validate_command_horizon.py` | 开发机Python3单关节理想时序实验与CSV/曲线；不是ROS控制节点 |
| `test/test_box_qp_solver.cpp` | GoogleTest解析解、零法向、约束缩放、预算与异常输入测试 |
| `test/test_box_qp_standalone.cpp` | 无ROS/gtest的解析与随机回归，独立检查约束、双向目标差及KKT |
| `test/qp_reference_cases.txt` | 固定2000组六维可行合成QP与参考目标值 |
| `test/qp_reference_cases.meta.json` | 固定参考集来源、数量、分组、SHA256及证据边界 |
| `test/test_qp_input_files.py` | 开发机Python3验证缺失、空、截断、非有限和数量不符文件必须失败 |
| `test/test_real_robot_command_gate.cpp` | GoogleTest命令检查与名称/时间规则测试 |
| `test/test_gate_standalone.cpp` | 不依赖ROS/Eigen/GoogleTest的命令与保持核心回归 |
| `test/test_bridge_callbacks.py` | Python3开发机编译实际网关方法，使用ROS桩验证放行/故障/保持；不代替ROS集成 |
| `test/test_stop_hold_adapter.py` | Windows/Linux编译实际适配器，使用ROS接口桩验证23场景76断言 |
| `test/test_command_horizon.py` | 解析样条、候选命令限幅、采样时序与停发模型测试 |
| `test/test_real_robot_tf_and_error.py` | URDF夹具、误差函数与launch接线的离线检查 |
| `test/test_real_robot_wiring.py` | 假反馈/假状态/假停止的ROS隔离夹具，禁止接真机master |
| `test/test_real_robot_wiring.test` | 隔离网关、监视器与模型发布器的rostest入口 |
| `docs/interfaces.md` | 所有业务接口和故障保持契约 |
| `docs/usage.md` | 编译、仿真、影子、现场操作和记录 |
| `docs/testing.md` | 测试层级、命令、判定与未验证边界 |
| `docs/qp_solver.md` | QP数学问题、算法、总预算、残差及复现说明 |
| `docs/stop_adapter.md` | stop接口、持续确认、失败路径、Windows与现场验证边界 |
| `docs/horizon_validation.md` | 离线模型、21组条件、简化审查与未改变的生产参数 |
| `docs/evidence/stop-horizon-windows-2026-10-25/` | 实际测试日志、原始模型轨迹、失败学习材料、代数审查及源码校验 |

## 验证边界

独立 C++ 测试验证命令与保持构造，Python 离线检查验证纯函数及接线文本。ROS 集成、实际 JTC 插值、Gazebo 性能、总线故障处理和真机停止保持需分别执行。

QP采用盒有效集和带区间保护的半空间乘子搜索，求解接口保持不变。独立数值验收覆盖40项解析断言、固定2000组和新种子2000组合成问题、10项真实GoogleTest及10项文件契约检查。测试在Windows/Zig C++14上执行，Eigen3.4.1和3.3.4均完成数值验证。旧Eigen与本机新版Clang组合需要仅用于本机兼容测试的延迟模板解析选项，不代表Ubuntu/GCC/catkin已验证。

`success`仅表示容差内可行，`converged`还须通过KKT检查。兼容字段`projected_gradient_norm`现在表示归一化KKT残差，不与旧投影梯度残差混比。合成测试比例不是现场故障率，开发机计时不是硬实时上限。完整ROS构建、Gazebo R1–R9与真机需分别验收；准确求解后关节居中及奇异区行为可能变化。

默认真机配置限值为空、现场审查关闭。站点 `stop_hold` 服务不由本包伪造；必须接入并验证真实停止保持动作。厂家 `disable_robot` 和 `stop_teleop` 不能直接冒充相同契约。总线中断时仍需已验证的执行层看门狗和现场独立停止设施。

可选stop适配器已完成Windows接口桩验证（23场景76断言），以及21组单关节时域模型和4项模型测试、14项既有离线检查。生产时域仍为2.5倍控制dt，容差仍5 mm，QP/关节参数/现有网关检查未改变。140 ms只是后续隔离A/B候选；这些通过结果不能作为现场停车和真机放行证据。
