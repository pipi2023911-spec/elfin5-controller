# 硬件实现与检查记录

日期：2026-10-13。状态：首版代码已实现；完整 ROS Melodic 构建、rostest、RViz 运行和真机验证待现场执行。不能将本记录中的离线 PASS 当作硬件验收。

## 已实现

- 复用 src_gazebo 原 V2：算法、头文件、原仿真 launch/YAML/测试脚本均未修改。
- 新增独立命令网关与纯 C++ 命令检查核心。默认 OBSERVE；READY 后仍需手动 enable；故障锁存，恢复输入不自动恢复 ACTIVE。
- 新增监视节点、移动基座模型生成、真实关节 RViz 显示和动态基座 TF 两种模式。
- 新增末端误差模长、各关节过程跟踪误差；使用 ROS rosbag 记录，离线导出 PNG、CSV 和统计。
- 新增离线单元测试、C++ gtest 与隔离 ROS 接线测试。
- README 写明现场七步流程、实际命令、站点接口契约与停止/重启处理。

## 相对设计方案的明确实现选择

1. 模型统一使用 /elfin5_real/robot_description，避免覆盖厂家全局参数。厂家固定 world 模型的 robot_state_publisher 仍必须退出，TF 名称不会因为参数命名空间自动隔离。
2. 录包复用 ROS 自带 rosbag，通过 README 的独立终端命令执行；monitor 不增加重复 rosbag 写盘线程。绘图时导出 CSV，原始 bag 是回溯依据。
3. 所有输入要求非零递增时间戳与持续发布，固定目标也有心跳要求；本版没有无限保留静态目标的特殊分支。
4. 使用就近基座样本与关节反馈对齐，限制最大时间差，不伪装严格同步；记录参与计算的各时间戳。
5. joint_tracking_error 话题使用 JointState 的 position 保存误差（rad），只作监视，不能接入 robot_state_publisher。原始 desired/actual/error 则从控制器 state 录包保留。
6. 新增 test/test_real_robot_wiring.py，作为 .test 的可执行测试夹具，放在 test 目录，没有增加业务节点。
7. 经2026-10-13简化复核，monitor仅负责观测，TF/误差诊断不再作为ACTIVE条件；控制反馈有效性仍由网关检查。详见文末分类依据。
8. 原有候选输出和诊断没有共同周期 ID，故本版不能证明严格逐周期来源。三次插值包络只是依据观测参考的保守筛查；这些局限保留在 README。
9. 现场硬件状态和停止适配器尚无可核实的接口信息。实现了可配置契约和失效阻断，没有伪造厂家服务或默认成功的适配器。

## 实际执行的检查

| 检查 | 结果 | 证据范围 |
|---|---|---|
| test_real_robot_tf_and_error.py | PASS，11 项（本轮复核） | 模型固定旋转保留、异常锚点拒绝、名称/符号/NaN、角度不隐式折叠、缺失段、ACTIVE覆盖率、launch输出重映射 |
| 纯 C++ 命令检查核心 | PASS | 临时独立 C++14 编译与断言执行；覆盖正常小步、旧/未来时间、重复名称、NaN、越界、步长、三次包络加速度、时域和双时钟新鲜度 |
| 新增 Python 文件语法 | PASS | Python 3.12 AST 与 Python 2 兼容语法解析；不是 Python 2.7 实际运行证明 |
| XML/YAML 解析 | PASS | package.xml、硬件 launch、ROS测试 launch、硬件 YAML、RViz YAML |
| 绘图/CSV/统计端到端 | PASS，合成数据 | 用 rosbag 接口替身提供80组样本，调用实际绘图入口并核对六轴数值及统计；PNG已目视检查。未伪造真机曲线，测试产物不交付为实验结果 |
| catkin_make / 原 C++ gtest / ROS接线rostest | 未执行 | 当前 Windows 无 ROS Melodic；现场必须编译执行 |
| 厂家xacro完整展开、ROS模型加载、FK/Jacobian对照 | 未执行 | 当前只有离线结构夹具检查，现场需用实际厂家包验证 |
| RViz/TF真实运行、控制器插值、时间对齐 | 未执行 | 未连接真机 |
| 停止/急停/看门狗/使能与低速运动 | 未执行 | 未提供现场适配器，默认禁止 ACTIVE |

独立 C++ 检查使用临时 Zig/Clang 工具链，不作为 ROS 项目依赖；编译器自带 libunwind 有 Windows 属性告警，命令检查源码未报告编译错误。绘图检查使用临时 Python 依赖，不改变目标机 Melodic 的依赖约定。

## 交付变更范围

首次硬件接入对原文件的修改范围为：CMakeLists.txt、package.xml、README.md、docs/HARDWARE_CONTROL_PLAN.md。本轮简化仅修改硬件网关、硬件launch、相关既有测试及配置注释/说明；原V2算法和厂家源码保持不变。
其余原文件逐项核对 SHA256。新增文件按 README 分配；不会向目标包复制临时编译器、Python依赖、合成数据、构建产物或 __pycache__。

从 README 的“现场第1步”开始。只有上述待现场项目完成并记录实际证据后，才能更新本记录的对应状态。

## 2026-10-13 按 Robot Control Development Rules 简化复核

任务归类：硬件接口/系统管理的局部简化及已有停止故障处理修正；不是重写算法或开发新的工业安全系统。未新增业务模块、节点、状态、ROS接口或自动恢复机制。

### 检查分类及处理依据

| 检查/逻辑 | 分类 | 源码依据与本次处理 |
|---|---|---|
| 硬件身份、OK诊断、反馈/JTC有效性、超时、显式运动放行 | A 安全关键 | bridge::onHardware/onJoint/onController/ready 保留；厂家专用诊断适配仍待完成，不能认定已独立读到急停/故障字 |
| 位置/速度/加速度、端点步长、旧命令拒绝、三次包络 | A 安全关键 | real_robot_command_gate.cpp::checkCommand 保留，不放宽阈值；力矩不由该位置命令网关控制，厂家侧保障尚未验收 |
| 控制器running、位置接口六轴资源、命令源及stop服务可用 | A 执行链保护 | bridge::poll 保留；ROS图快照不是持续权限锁，但尚无厂家替代证据，不能删除 |
| 坐标系、数值/维度、QP结果状态 | B 算法必要 | 原V2及网关入口保持；QP_DEGRADED未收敛仍阻止放行，没有凭可行标志放宽 |
| QP与网关各自的输入检查；纯函数的限值校验 | A/B 边界检查 | 没有机械地去重：原QP还独立服务仿真，网关承担真实输出边界；公共纯函数不能假定调用者先校验 |
| monitor诊断必须OK且新鲜、显示节点required | C 非必要控制耦合 | 已移除。QP::controlTimerCallback通过poseToIsometry(base_pose.pose)直接构造变换，目标用其逆变换，不读TF或monitor结果；网关的六类控制输入、关节/基座/JTC时间差仍保留 |
| monitor自身的输入合法性、TF冲突及误差对齐 | B 观测数据必要 | 保留，用于显示和实验数据有效性；其告警不再锁存运动状态 |
| active_wall_ | C 无效存储 | 只有声明与赋值，没有读取；删除，候选看门狗继续使用last_candidate_wall_ |

分类中的安全关键是本项目工程职责，不表示此ROS节点达到安全认证等级。RViz/记录不可用时数据质量由操作员判断；本次解除显示依赖意味着显示退出时有效控制可以继续，操作员需要停止实验时显式关闭网关。

### 复核发现的具体问题及最小修正

| 条件及位置 | 观测值/阈值 | 实际原因 | 修改与影响 |
|---|---|---|---|
| 原bridge::ready/onMonitor要求monitor有效 | 源码要求level=OK且年龄≤input_timeout（默认0.3s）；无现场观测数据 | 把显示TF/误差质量当作控制输入；观测启动延迟也可能导致控制锁存 | 移除此一项门禁及订阅；原始控制反馈失效仍停机 |
| 原bridge::enable(false)直接进入FAULT_LATCHED | ACTIVE下的人工正常关闭；不是阈值超限 | 正常操作和故障处理共用故障状态 | 正常关闭回OBSERVE，待停止完成可READY；没有自动恢复运动 |
| 原bridge::reset只检查停止是否仍在调用 | 停止服务通信失败或response.success=false后，原代码允许reset | 丢失了停止失败这一结果 | 保留stop_failed_，拒绝reset；仅显式enable(false)重试一次，成功后仍需人工reset；不把回执当作物理停止证明 |
| 原bridge::fault会覆盖已锁存原因 | 后续任何异常输入回调；无数值阈值 | 第一故障和停止失败文字可能被覆盖 | 锁存后保留首因，停止线程追加调用结果，人工reset后清除 |

### 用户确认的待补证据（2026-10-13）

用户明确答复：目前没有joint_states、JTC state、base_pose的实测频率、最大延迟，也没有厂家停止/看门狗说明，只有源码；之后查到会提供再修改。此记录是项目待办，不是已验证的现场结论。

| 参数/证据 | 当前源码值 | 验证状态与后续动作 |
|---|---|---|
| input_timeout | bridge/monitor/QP均0.3s | 未实测；补六类控制输入周期、源时间与接收延迟、最大中断时间 |
| max_input_skew | bridge/monitor均0.08s | 未实测；补设备时钟同步、关节-定位及关节-JTC时间差分布 |
| candidate_max_age | 0.02s | 未实测；补QP到网关传输/调度年龄及峰值 |
| candidate_timeout | 0.15s | 未实测；补候选周期、最长中断及现场允许停止响应时间 |
| min_horizon / max_horizon | 0.02s / 0.15s | 未验证真实JTC插值；补参考采样和轨迹接收、执行时间证据 |
| 独立停止/失联看门狗 | 无现场说明与测试记录 | 补厂家动作/状态接口、停止回执语义、断网与进程退出响应；不造固定OK或空stop服务 |

上述数值本次均未改变，仅供现有离线/影子起始配置；不得解释为已核准真机阈值。site_review_complete仍默认false，真实关节限值仍为空，当前交付不具备直接运动准入条件。拿到数据后，先记录阻断位置、实际值、阈值、A/B/C分类及原因，再只调整对应参数/代码。

### 本轮实际验证及范围

- Python离线测试：11项通过，包括原有模型/误差检查及显示节点退出不终止控制链的launch结构检查。
- C++14编译及离线运行：纯命令检查断言通过；从实际bridge源码提取的ready/enable/reset/fault/tick/stopLoop函数原样编译，在假时钟/假停止服务下验证六路输入、正常关闭不自动恢复、停止通信/业务失败阻止复位、显式重试、首因保留、输入/候选超时锁存。
- Python AST和XML解析通过。未改算法源码；算法输出未重新仿真或实测，本次不能报告新的收敛性/稳定性结论。
- 已扩充现有ROS隔离测试：monitor缺失/WARN/停发不影响有效控制；真实反馈中断仍锁存；硬件故障、停止失败、人工重试和reset流程。当前环境无ROS Melodic，完整节点编译与该rostest尚未执行；函数替身测试不代替ROS回调/线程时序验证。
- 尚待现场：catkin构建/rostest、真实TF/RViz、厂商诊断及停止适配、全部时间参数、JTC插值/限值、实际停止/看门狗与低速运动。没有启动主站、使能、回零或发送真机轨迹。

