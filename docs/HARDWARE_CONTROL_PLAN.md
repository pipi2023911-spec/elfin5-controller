# Elfin5 V2 EtherCAT 真机控制与 RViz 显示方案

设计日期：2026-10-12；实现更新：2026-10-13。首版硬件接入代码已实现，原 V2 算法未修改。ROS 构建和真机验证尚未执行；实际检查范围见 [HARDWARE_IMPLEMENTATION_STATUS.md](HARDWARE_IMPLEMENTATION_STATUS.md)。
唯一目标包：[elfin5_qp_controller_code_quality_v2](../)。
ROS包名保持elfin5_qp_controller。所有新增源码、节点、配置和测试均放入此包已有目录，不再创建平行包或复制V2。

## 1. 代码文件分配（首先确定）

下列分配已落实。src新增硬件接入代码；src_gazebo中3个V2实现原样复用，不复制到src。

```text
elfin5_qp_controller_code_quality_v2/
├── src_gazebo/                         [已有，保留原内容]
│   ├── box_qp_solver.cpp               复用求解器
│   ├── qp_cartesian_controller.cpp     复用V2算法、运动学与轨迹生成
│   └── qp_cartesian_controller_node.cpp 复用现有节点入口
├── src/                               [仅新增硬件集成代码]
│   ├── real_robot_bridge_node.cpp      新增：候选轨迹隔离、状态机、命令放行
│   ├── real_robot_command_gate.cpp     新增：输入/候选轨迹检查，不重复QP求解
│   └── real_robot_monitor_node.cpp     新增：真实状态、基座TF、RViz标记、末端及关节误差
├── include/elfin5_qp_controller/
│   ├── box_qp_solver.h                 已有，不改
│   ├── qp_cartesian_controller.h       已有，不改
│   └── real_robot_command_gate.h       新增：可离线测试的网关契约
├── config/
│   ├── elfin5_qp_controller.yaml       已有，保留，作为V2参数基底
│   ├── qp_test_suite.yaml              已有，保留，仅用于原仿真测试
│   ├── elfin5_real_robot.yaml          新增：话题映射、门禁、日志与已核验限制
│   └── elfin5_real_robot.rviz          新增：RobotModel、TF、目标、实测估计轨迹
├── launch/
│   ├── elfin5_qp_controller.launch     已有，不改
│   ├── elfin5_qp_gazebo.launch         已有，不改
│   └── elfin5_qp_real_robot.launch     新增：模型、候选QP、网关、监视与RViz
├── scripts/
│   ├── run_qp_test_suite.py            已有，不改；不作为真机启动/记录入口
│   ├── prepare_real_robot_description.py 新增：生成移动基座用模型描述到标准输出
│   └── plot_real_robot_error.py        新增：末端误差模长与六关节过程跟踪误差曲线
├── test/
│   ├── test_box_qp_solver.cpp          已有，不改，不另造同类测试
│   ├── test_real_robot_command_gate.cpp 新增：名称/限位/时效/模式门禁
│   ├── test_real_robot_wiring.test     新增：隔离话题、禁用/重启零放行
│   ├── test_real_robot_wiring.py       新增：隔离ROS测试夹具
│   └── test_real_robot_tf_and_error.py 新增：动态基座TF、末端及关节误差重算
├── docs/
│   ├── [现有5份文档]                  保留内容，不覆盖已有测试结论
│   ├── HARDWARE_CONTROL_PLAN.md        本方案，硬件集成的设计入口
│   └── HARDWARE_IMPLEMENTATION_STATUS.md 实际实现选择与验证状态
├── CMakeLists.txt                      已调整构建路径并增加硬件目标
└── package.xml                         已补硬件接入所需依赖
```

节点与文件一一对应：

| 节点/执行项 | 文件/来源 | 作用 |
|---|---|---|
| elfin5_qp_candidate | 复用现有elfin5_qp_controller_node可执行文件 | V2只生成候选轨迹，输出强制重映射到候选话题 |
| elfin5_real_bridge | src/real_robot_bridge_node.cpp | 唯一被本方案允许写厂家command话题的节点 |
| elfin5_real_monitor | src/real_robot_monitor_node.cpp | 只读取状态；提供动态基座TF、目标与轨迹标记、末端及关节误差 |
| robot_state_publisher | ROS已有节点 | 用真实joint_states发布关节TF；不重写此功能 |
| rviz | ROS已有节点 | Fixed Frame=world，显示真实反馈驱动的模型 |
| plot_real_robot_error.py | 离线脚本，不是控制节点 | 读取记录，输出E(t)、六关节e_q,i(t)曲线和统计 |

不新增第二个EtherCAT主站、第二套RobotHW、第二个QP求解器或重复的轨迹积分模块。V2已有积分与JointTrajectory组装继续复用；command_gate只审查，不重新积分、不静默修改端点/时间。

## 2. 已核实的现有文件及允许修改范围

- [CMakeLists.txt](../CMakeLists.txt)初始引用src/下3个不存在的原控制器实现，实际文件在src_gazebo/。本次已修正源文件路径，保留原库/可执行目标名称，增加3个新硬件源文件对应目标与安装项；不移动原文件。
- [V2接口](../src_gazebo/qp_cartesian_controller.cpp#L151)已提供joint_states、base_pose、target_point、轨迹输出及enable服务，直接复用。
- [V2异常路径](../src_gazebo/qp_cartesian_controller.cpp#L826)在enabled检查前可能发布hold。因此只将enabled设false不足以隔离真机，必须把全部输出先导向候选话题。
- [既有测试脚本](../scripts/run_qp_test_suite.py#L1)包含MoveIt运动和Gazebo实验职责，不能直接拿来做只读真机记录。
- V2已有时域统一、FK/Jacobian坐标转换、求解器收敛区分，不重复实现。
- 原有src_gazebo、头文件、launch、YAML、测试脚本和测试结果均不改。厂商包也不改。README按用户要求增加现场操作流程。
- package.xml本次按实际功能补control_msgs、tf2_ros、visualization_msgs、nav_msgs、robot_state_publisher、rviz、xacro以及绘图依赖；controller_manager_msgs已经存在，需按C++使用情况补正确的构建依赖范围。
- 新增脚本与ROS Melodic环境兼容；不因开发机可用Python3就假定目标机运行时一致。
- 当前包无Git元数据；实施前保留原文件SHA256基线，实施后核对只有白名单中的构建文件及新增文件变化。

若以后发现必须修改V2算法或其接口，先以具体证据提出单独的变更清单，不通过复制一整套控制器来规避审查，也不在本次硬件接入中暗改原代码。

## 3. EtherCAT 接入路径

```text
真实关节反馈 + world中的base_pose + world中的target_point
                ↓
   elfin5_real_bridge：校验输入、转发到候选QP专用输入
                ↓
   原V2节点 elfin5_qp_candidate
                ↓
   /elfin5_real/candidate_trajectory
                ↓
   elfin5_real_bridge：命令状态机与轨迹审查
                ↓
   /elfin_arm_controller/command
                ↓
   厂家 JointTrajectoryController（位置接口）
                ↓
   ElfinHWInterface → 原EtherCAT主站 → 伺服驱动器
```

候选节点对绝对名称/joint_states和/elfin_arm_controller/command也必须使用明确ROS remap，不只改变节点namespace。启动检查核对实际解析后的发布/订阅关系；remap缺失或候选节点直接连接真实command时拒绝放行。

候选话题是部署隔离措施，不是ROS1访问权限机制。其他节点仍可能直接访问厂家topic/action，须通过部署管理保证QP、MoveIt Execute、手工轨迹和其他执行器不会同时发命令。图上只有一个网关不等于系统天然拥有强制访问控制。

## 4. 输入、输出与状态机

| 接口 | 类型/语义 | 规则 |
|---|---|---|
| joint_states输入 | sensor_msgs/JointState，真实六轴反馈 | 参数指定真实来源；按名称重排，不采用仿真或joint_state_publisher假数据 |
| base_pose输入 | PoseStamped，base在world中的位姿 | frame_id=world；确认具体base link与时间来源 |
| target_point输入 | PointStamped，目标XYZ在world | 保留目标编号/变更时刻；静态目标与失联规则明确区分 |
| candidate_trajectory | JointTrajectory | 私有候选通道，非锁存，队列有界 |
| 实际运动输出 | 厂家command话题 | 只有ACTIVE且所有门禁满足时允许转发 |
| 控制器反馈 | JointTrajectoryControllerState及ListControllers | 核对running、接口和六轴资源；仅查询，不自动切换 |
| 厂家故障/使能状态 | 实际发现的话题或只读接口 | 名称参数化；未知、过期、缺失均不能视为正常 |
| tracking_error | Float64及带时间/有效性诊断 | 末端误差模长E(t)，单位m；绘图转mm |
| 关节过程误差记录 | 控制器state中的header、joint_names、desired、actual、error | 按关节名提取同一状态消息的期望/实际位置，记录六关节有符号位置跟踪误差；rad存储、°绘图 |
| RViz数据 | TF、MarkerArray、Path | 全部来自真实反馈或其模型估计，不用于发送控制命令 |

网关沿用 OBSERVE → READY → ACTIVE；控制关键异常进入 FAULT_LATCHED，数据恢复不会自动恢复 ACTIVE。人工正常关闭回到 OBSERVE 并请求停止，停止请求完成前不允许放行；停止成功后可回到 READY。停止失败仍锁存，只有再次人工关闭才重试停止，成功后再人工 reset。成功回执不证明物理静止。

具体门禁：
1. 模型/关节身份、frame_id、数组长度、重复关节名、有限数、时间戳及接收年龄符合契约。
2. 控制器状态、硬件状态、指令所有权与已核验参数齐全；配置缺失则只允许观察。
3. 候选命令产生时间晚于本次放行代次，无旧队列回放；名称、尺寸、位置/速度/时间参数有效，命令连续性及插值约束经过专项检查。
4. 失效时先关闭继续放行，锁存异常，再请求已验证的下层停止动作。停止发布不等于驱动器已经停止；未明确下层停止/看门狗接口时不进入真机ACTIVE。
5. 不自动调用厂家enable_robot、clear_fault、模式切换、位置识别或刹车接口。
6. monitor 的 TF/误差诊断仅用于显示与数据质量判断，不作为运动门禁。QP 使用输入 base_pose 与关节反馈，不消费 monitor 或 RViz TF；控制输入检查仍在网关及原算法边界保留。monitor/模型显示节点退出不会通过 required 终止控制链。

V2候选轨迹没有源输入快照ID，诊断与轨迹也不是原子配对。网关可记录发送/接收及其自身输入快照，但不能虚构“这一轨迹精确对应哪次QP输入”。不使用某条最新诊断作为命令安全的唯一依据。若放行标准需要严格逐周期来源证明，保持该项未关闭，另列最小接口变更，不擅自改V2。

“候选输出隔离”和“独立硬件安全停止”是不同层级。第一版先验证隔离、输入审查与只读显示；网关不被宣传为安全认证控制器。

## 5. 厂家驱动启动与配置边界

已有厂家主站和控制器继续使用，不从新launch中隐式启动。现场按已审查流程准备硬件后，本包才附着到运行系统。

已知前置问题：
- 自动位置识别（厂家源码 `elfin_robot/elfin_ethercat_driver/src/elfin_ethercat_driver.cpp:238`）默认开启。
- 客户端构造（厂家源码 `elfin_robot/elfin_ethercat_driver/src/elfin_ethercat_client.cpp:96`）写SDO及控制字/模式。
- 硬件写入（厂家源码 `elfin_robot/elfin_ros_control/src/elfin_hardware_interface.cpp:617`）用effort反馈生成目标力矩字段。
- 厂家控制启动（厂家源码 `elfin_robot/elfin_robot_bringup/launch/elfin_ros_control.launch:10`）加载arm controller不等于已运行。

因此，读取参数/订阅状态与启动主站严格区分。真实网卡、零位、减速比、ESI/固件、力矩字段语义及停止机制未核实前不放行运动；不复制厂家example YAML来填空。

## 6. 真机在 RViz 中显示：模型、关节和基座三者一致

RViz不是控制仿真器。关节姿态使用真机joint_states，基座位置使用真实base_pose，目标点使用world输入。无需启动Gazebo、fake controller、joint_state_publisher GUI或MoveIt Execute。

期望TF树：

```text
world
  └── elfin_base_link                外部定位的动态TF，唯一发布者
        └── elfin_base               模型固定变换，保留厂家90°旋转
              └── elfin_link1 ... elfin_end_link   真实关节反馈驱动
```

厂家模型（厂家源码 `elfin_robot/elfin_description/urdf/elfin5.urdf.xacro:14`）包含固定world_base关节；若原样发布，再另发动态world→elfin_base_link，会产生冲突。

处理方案：
1. prepare_real_robot_description.py调用厂家xacro生成URDF，在内存中核验并移除准确匹配的world_base固定关节及其孤立world链接，输出以elfin_base_link为根的描述。只处理已核实结构，结构不符则报错；不编辑厂家xacro、不复制整套URDF或mesh。
2. 保留elfin_base_link→elfin_base固定旋转、全部关节与几何参数。复用厂家SRDF和关节限制配置；只加载模型参数，不启动move_group执行功能。
3. /elfin5_real/robot_description由一个启动入口管理，保留厂家全局参数，RViz、V2及monitor使用一致的几何和link语义。已经存在冲突模型或TF发布者时报告冲突，不在控制运行中静默覆盖。
4. robot_state_publisher只负责模型内部TF，使用真实关节反馈。不要再同时运行厂家elfin5_bringup.launch中的另一套固定world模型发布器。
5. 若定位系统已发布正确的world→base TF，monitor只监听；否则由monitor按base_pose发布。配置模式只能二选一，并核对TF树、重复发布者与实际link名称。
6. 发布保留定位测量时间，不将过期位姿改写为now伪装新鲜；定位失效时显示状态，轨迹标记失效/停止更新，不能用最后一帧“仍能显示”表示状态正常。

RViz配置：
- Fixed Frame：world。
- RobotModel：真实反馈姿态；TF：world、base、TCP。
- Marker：目标点、估计实际TCP、两者连线。连线用于空间观察，不绘制XYZ误差分量曲线。
- Path：实际末端world轨迹，按有界时间窗/点数保留。
- 状态文字：OBSERVE/ACTIVE/FAULT、反馈年龄、QP状态及误差模长。
- 不启用交互式运动目标、轨迹Execute等会产生控制请求的功能。

V2本身已经把FK转成base相对坐标，生成base根模型后仍需回归验证相对FK与Jacobian数值一致，避免多转或漏转厂家固定90°关系。能显示模型不代表位姿准确，需比对TF位置与独立FK。

## 7. 误差曲线：末端误差模长与各关节过程跟踪误差

### 7.1 末端位置误差模长

末端误差只画标量：

E(t) = ||p_target_W(t) − [b_W(t) + R_WB(t) FK_B(q_actual(t))]||₂

横轴：时间s；纵轴：E(t)×1000，单位mm。无XYZ三轴误差图，也不做三轴RMSE面板。内部向量运算只是计算E(t)所需，不代表新增三轴输出需求。

曲线标注目标更新、基座移动和异常事件；在容差确定后画一条标量阈值线。给出E的RMSE、P95、最大值、超限时长、有效覆盖率，以及适用时的持续到达时间。

monitor用同步的真实joint_states、base_pose与target重算E，不直接把无时间戳Float64拼成测量真值。原始状态和时间戳随记录保留，缺失/跳变区间断开，不补零。位置是基于模型和定位的估计；物理TCP精度另需外部参考。

RViz显示标量误差数值、目标与末端空间轨迹。历史E(t)曲线由plot_real_robot_error.py生成；如需在线观察，使用已有rqt_plot订阅标量话题，不为本轮单独开发RViz绘图插件。

monitor负责计算并发布带时间/有效性的数据；原始记录复用独立rosbag进程，原Gazebo测试脚本不启动。保存原始rosbag、末端标量记录、关节期望/实际/误差记录和实验元数据；记录目录由README录包命令明确指定，结果不写入src、src_gazebo或配置目录，磁盘写入不得阻塞控制回调。


### 7.2 六个关节的过程位置跟踪误差

对每个关节i=1…6，在整个运动过程中定义：

e_q,i(t) = q_desired,i(t) − q_actual,i(t)

q_desired是下层JointTrajectoryController在该状态采样时刻执行的插值参考位置；q_actual是同一控制器状态消息中的实测反馈位置。这衡量参考轨迹的关节执行跟踪情况，不用最终目标关节角或QP未来轨迹端点减当前角度来冒充过程误差。

数据源规划为/elfin_arm_controller/state（control_msgs/JointTrajectoryControllerState）；实际话题名称和消息字段在现场只读阶段核实，参数化配置。按joint_names映射六轴，使用同一条消息的header.stamp、desired.positions与actual.positions，不拿两个不同时间的“最新值”直接相减。原始error.positions也保留，用于交叉核对控制器误差语义。

ROS1上游[JointTrajectoryController实现](https://github.com/ros-controls/ros_controllers/blob/noetic-devel/joint_trajectory_controller/include/joint_trajectory_controller/joint_trajectory_controller_impl.h)可作为desired/actual/error状态及角差处理的参考；它不是当前工控机Melodic部署版本的证明。实施时核对实际版本的角度处理：对有限位关节使用一致零位和角度分支的直接差；不能盲目把所有差值折叠到±π而掩盖跳变。连续关节若存在，则明确采用展开角差或最短角差，并单独标注。与控制器error字段语义不一致时保留两者并说明原因，不静默替换。

绘图规范：
- 一张图包含J1～J6六个子图（3×2），共享时间轴；每个子图画一条有符号e_q,i(t)，纵轴单位°，内部仍以rad记录。
- 保留正负方向和零线，避免仅画绝对值掩盖超前/滞后方向；阈值在逐轴确定后画±容差线。
- 统一标注目标切换、基座运动、启停及异常事件，与末端E(t)图使用同一时间起点。
- 每轴统计RMSE、最大绝对误差、绝对误差P95、超限时长和有效覆盖率；统计仅覆盖标注的有效运动区间。
- 保存每轴期望和实际位置，便于追查偏差；本次必需新增的是六轴位置误差曲线，不默认扩展速度/力矩误差图。

缺失关节、数组异常、时间回跳及过期数据均标为无效并断开曲线，不填零、不伪造更高采样率。控制器state发布频率决定曲线可观测的带宽，必须记录实际频率和丢帧情况；低频曲线不能证明伺服周期内没有误差峰值。state不可用时将关节误差标记为不可用，不用候选命令端点替代参考位置。

此功能由real_robot_monitor_node.cpp监视、独立rosbag记录、plot_real_robot_error.py导出CSV并绘制，无需增加节点或另建绘图脚本。test_real_robot_tf_and_error.py补充关节名称乱序、符号与单位转换、角度分支、缺失数据和时间异常的离线校验。

两类误差联合观察：E(t)反映world坐标中的末端任务跟踪，e_q,i(t)反映下层关节参考的执行跟踪。六轴误差很小而末端误差较大时，应继续检查上层参考生成、基座定位、时间对齐和模型；某轴误差持续较大时，应进一步检查该轴参考变化与执行响应。这些是排查方向，不能仅凭曲线直接归因。

## 8. 移动基座控制的性能边界

已确认基座移动且目标以固定world表达。V2只做位置反馈，未实现显式基座线/角速度前馈；在容差内会发关节hold，关节hold不能保证基座继续移动时TCP保持world位置。

本次硬件接入复用V2，不暗改算法。先测静止、平移、旋转和组合运动的E(t)及关节响应，记录V2基线。若任务要求更高动态精度或无死区持续保持，再以数据提出前馈/到点策略的独立算法变更；不能宣称本次接线已解决动态跟踪性能。

姿态自由不表示工具姿态、碰撞和关节限位可以忽略。移动基座与机械臂的联合停止由现场系统确认，不假定此包有移动底座控制权限。

## 9. 实施顺序与验收

| 步骤 | 内容 | 验收证据 |
|---|---|---|
| A | 记录原文件哈希，修正构建路径并增加硬件目标 | 原算法源码不变，新增文件与本清单一致 |
| B | 网关离线测试与ROS隔离测试 | OBSERVE/FAULT/重启/候选hold均不触达真实command |
| C | 模型生成与RViz离线验证 | 无重复world→base；相对FK与原模型一致；真实/回放输入改变对应TF |
| D | 影子计算与记录 | QP只发布候选；只读monitor产生E(t)、关节误差记录和RViz数据；缺失段正确标识 |
| E | 现场只读附着 | 实际控制器、关节映射、定位与TF来源核验；记录运行版本 |
| F | 硬件门禁与停止验收 | 所有运动阻断项关闭；独立停止机制及命令唯一性有证据 |
| G | 现场授权受控运动 | 小范围逐级试验，RViz真实反馈、末端误差及六关节过程误差日志齐全 |

本次已实现A～D相关代码与测试夹具，并完成部分离线检查；没有实际执行的ROS编译、运行及E～G真机条目均不标PASS。以HARDWARE_IMPLEMENTATION_STATUS.md为当前验证状态。本方案是现阶段硬件方案入口，取代此前“XYZ三轴误差曲线”的设计。


## 10. 现场入口

先阅读README中的七步现场流程。当前默认配置为观察/影子计算；站点硬件状态与停止适配、真实限值及现场审查未完成前，不允许ACTIVE。具体实现选择、与方案的差异及实测范围统一见HARDWARE_IMPLEMENTATION_STATUS.md。
