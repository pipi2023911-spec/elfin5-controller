# 分层测试与验收

测试代码和厂家驱动必须分开运行：以下合成反馈只能在隔离测试master使用。离线、编译、ROS接线、Gazebo和真机是独立完成等级。

## 1. 不依赖ROS的检查

先验证QP，仓库根目录执行（需要Eigen头文件）：

```bash
g++ -std=c++14 -O2 -Wall -Wextra -Wpedantic -I include -isystem /usr/include/eigen3 \
  src_gazebo/box_qp_solver.cpp test/test_box_qp_standalone.cpp -o /tmp/elfin5_qp_test
/tmp/elfin5_qp_test test/qp_reference_cases.txt 2000
python3 test/test_qp_input_files.py /tmp/elfin5_qp_test
```

程序先执行40项解析断言，再要求完整读取指定数量的参考用例，独立检查有限值、原始约束、双向目标差、KKT和收敛标志。缺失、空或损坏文件必须非零退出。生成新种子的命令、参考集溯源及计时边界见[QP算法](qp_solver.md)。独立计时不是整个控制周期，也不是最坏实时保证。

在仓库根目录，使用C++14编译器：

```bash
mkdir -p /tmp/elfin5_core_test
g++ -std=c++14 -Wall -Wextra -Wpedantic -I include \
  src/real_robot_command_gate.cpp test/test_gate_standalone.cpp \
  -o /tmp/elfin5_core_test/gate
/tmp/elfin5_core_test/gate
python test/test_real_robot_tf_and_error.py -v
```

独立C++包含29项断言：保持采用实际反馈、零端点速度、新时间/正时域、NaN/越界/回拉/过期/重放拒绝、名称映射和中文原因。它运行实际纯C++检查实现，不模拟机械臂。

Python检查涵盖模型结构夹具、原样保留URDF、误差映射/符号/缺失段、launch候选隔离及直接输入接线。语法解析不能证明ROS依赖或节点行为。

开发机有Python3及g++时还可执行：

```bash
python3 test/test_bridge_callbacks.py
```

该测试生成临时ROS类型/时钟/服务桩，编译并运行实际网关方法体，验证放行、故障锁存、一次保持、反馈超时/驱动故障不回拉、停止失败/人工重试、缓存目标冻结和查询期间关闭优先。可通过`--zig /path/to/zig`改用Zig C++编译器。测试公开成员并跳过ROS注册构造，不验证真实ROS API/ABI、节点注册、调度或物理停车，不能替代rostest。

## 2. 完整ROS构建与测试

Ubuntu18.04 / Melodic工作空间中：

```bash
cd ~/catkin_ws
catkin_make -DCATKIN_ENABLE_TESTING=ON
catkin_make run_tests_elfin5_qp_controller
catkin_test_results build/test_results
```

独立运行接线夹具：

```bash
source ~/catkin_ws/devel/setup.bash
rostest elfin5_qp_controller test_real_robot_wiring.test
```

rostest使用测试master，未启动EtherCAT或实际控制器。夹具生成合成JointState、JTC state、厂家Bool状态与假停止回执。验证：
- 未放行无输出，人工放行才有输出；
- 反馈超时锁存，不透传后续候选；
- 恢复数据不自动运动，reset不自动放行；
- 人工关闭也锁存并调用停止；
- 驱动故障不使用旧位置回拉；
- 停止服务失败禁止reset，人工false明确重试；
- 非法候选不透传，仅请求一次来自实际反馈的零端点速度保持；
- monitor的TF和误差估计、显示失效不参与网关放行。

合成状态不能用于现场准入，假服务成功不能解释为停车已验证。保留完整ROS编译/rostest日志，任何失败标记FAIL或未验证，不修改阈值掩盖失败。

## 3. Gazebo实验

按[操作文档](usage.md)运行Gazebo入口和R1。该入口验证QP→仿真JTC控制链，不加载真机bridge；网关状态行为由上面的隔离rostest验证。两种结果不得混为同一验收。

先重复R1，再选择换目标、约束、奇异和超时轮次。验收至少包括：
- 同一初始姿态、目标、版本和参数可重复；
- 原始bag/CSV及状态记录完整；
- 末端误差、达到容差情况及失败原因；
- 关节命令与JTC参考/反馈；
- 位置、速度、加速度约束分别检查，不把端点命令差分等同实际加速度；
- QP收敛/失败状态，不把可行未收敛当完整求解成功。

目前自动脚本保存误差范数及迭代/残差，未自动提供XYZ分量、求解耗时和完整时序统计。需要补齐原始字段后再报告对应指标。
求解器数值回归已覆盖盒/半空间反例、缩放、预算和独立KKT检查；Gazebo仍须重新运行，不能从数值测试推断轨迹不变。无活跃限制时的关节居中行为、活跃约束及奇异区恢复都要记录。R5/R6按各自到达/改善要求，R7/R8才使用配置中的允许安全终态，R9检查超时处置；不要把R5–R8套成同一种判据。

## 4. 真机分阶段验收

| 阶段 | 输出权限 | 需要证据 |
|---|---|---|
| 模型/接口观察 | 网关关闭 | 模型、关节名称/方向、反馈来源/周期、TF |
| 影子计算 | 网关关闭 | 候选与实际command隔离、QP状态、录包 |
| 停止保持核验 | 按现场授权独立执行 | 实际JTC/停止动作、负载、位置/速度、响应时间、失联行为 |
| 受控闭环 | 人工明确放行 | 小范围低速跟踪、真实误差/约束和控制时序 |
| 故障验证 | 先模拟，现场另行授权 | 不自动恢复、停止保持结果、独立停止和看门狗 |

最近有效反馈姿态只表示数据/限位检查通过，不能证明碰撞或设备健康。反馈超时和驱动故障时，网关可能无法直接实施位置保持；只有现场停止保持链及执行层保护能对实际行为提供证据。

厂家源码目标：RobotHW和EtherCAT循环各1ms、JointState50Hz、JTC state25Hz、厂家状态10Hz；均不是实测。核对现场软件版本/部署配置；总线缓存可能使上层话题继续更新，不能以话题频率证明PDO时效。

## 5. 记录格式

每轮保存TASK（目标、已验证基础、文件、最小方案、验收）与TASK_RESULT（实际修改、命令、结果、PASS/FAIL/未验证、日志/曲线、问题、下一步）。

不要将README中的操作步骤、测试夹具或计划当成已经执行的实验。独立核心测试通过不代表完整ROS构建通过；仿真通过也不代表真机停稳或性能已验证。
