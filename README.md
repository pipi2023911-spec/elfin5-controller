# Elfin5 V2：EtherCAT 真机接入、RViz 与误差记录

## 本次上传说明（2026-10-14）

本仓库发布 Elfin5 QP 控制器 V2 完整 ROS 包，面向控制算法研究、仿真和真机实验接入。

- QP 求解器采用带约束的投影梯度法，复用已有 V2 控制算法；本轮主要简化硬件接入逻辑，没有修改 QP 数学核心。
- 末端仅跟踪 XYZ，允许姿态变化；基座可移动，基座位姿和目标点均在固定 world 坐标系中表达。
- 保留真实反馈、硬件互锁、JTC 检查和关节限值；解除误差显示与运动准入的耦合，区分正常关闭与故障锁存，停止失败禁止直接复位。
- 默认启动 RViz，提供末端位置误差范数和六个关节过程跟踪误差。
- 发布前重新执行 11 项 Python 离线检查；已有 C++ 离线验证范围见检查记录。完整 ROS 编译、隔离 rostest 和真机试验尚未完成。
- 现场硬件诊断、停止服务及通信阈值仍待核实；默认配置只允许观察和影子计算，不能直接放行真机运动。

源码位置：[QP求解器](src_gazebo/box_qp_solver.cpp)、[笛卡尔控制器](src_gazebo/qp_cartesian_controller.cpp)、[硬件命令网关](src/real_robot_bridge_node.cpp)。

厂家依赖：[elfin_robot 的 melodic 分支](https://github.com/huayan-robotics/elfin_robot/tree/melodic)。本仓库不包含厂家驱动及设备专属标定配置，需与实际设备版本核对。

将本仓库克隆到已有 catkin 工作空间的 src 下（同一工作空间只保留一份 elfin5_qp_controller 包）：

```bash
cd ~/elfin_ws/src
git clone https://github.com/pipi2023911-spec/elfin5-controller.git
```

随后按下面的中文现场步骤编译和检查。文末折叠的 V1 说明为历史仿真资料，当前硬件行为以本说明及[硬件实现检查记录](docs/HARDWARE_IMPLEMENTATION_STATUS.md)为准。

本包只控制末端 XYZ，姿态自由。world 固定；base_pose 表示 elfin_base_link 在 world 中的位姿；target_point 也是 world 坐标。基座可以移动。

**现场从下面的流程开始，不要使用文末的 Gazebo 启动步骤。** 原 V2 求解器和控制器仍在 src_gazebo/，没有复制或修改。硬件接入使用厂家 EtherCAT 主站、RobotHW 和 position_controllers/JointTrajectoryController。默认配置只能观察/影子计算，不能放行真机运动。

实现日期：2026-10-13。当前 Windows 环境已做的检查与尚未执行的 ROS/真机检查见 [硬件实现检查记录](docs/HARDWARE_IMPLEMENTATION_STATUS.md)。已有 Gazebo 测试结果不代表硬件验证通过。

## 现场第 1 步：整理工作空间与编译

适用目标环境：Ubuntu 18.04、ROS Melodic、系统 Python 2.7。Windows 的微信目录是交付目录，不是在 Windows 上启动 EtherCAT。

将厂家 elfin_robot-melodic 中各 ROS 包和**本 V2 包**放进同一个 catkin 工作空间 src 下。目录名可以保持交付名称，ROS 包名仍是 elfin5_qp_controller。不要同时放入另一份同名 V1 包。确认厂家包与现场实际部署版本一致。

以下用 ~/elfin_ws 举例，请替换为现场路径：

~~~bash
source /opt/ros/melodic/setup.bash
cd ~/elfin_ws
rosdep install --from-paths src --ignore-src -r -y
catkin_make
source devel/setup.bash
rospack find elfin5_qp_controller
~~~

rospack 返回的路径必须是本次 V2 包。构建已修正为引用 src_gazebo/，无需手动移动源码。安装空间也安装了两个新节点、模型生成/绘图脚本和配置。

先在**无真机连接的测试环境**验证：

~~~bash
cd ~/elfin_ws
catkin_make run_tests_elfin5_qp_controller
catkin_test_results build/test_results
python "$(rospack find elfin5_qp_controller)/test/test_real_robot_tf_and_error.py" -v
rostest elfin5_qp_controller test_real_robot_wiring.test
~~~

纯 Python 检查命令使用源码空间。ROS 隔离测试使用合成数据和独立测试 master；不要把 test_real_robot_wiring.py 直接 rosrun 到真机 master。测试通过后再进入现场只读流程。完整 ROS 构建和 rostest 尚未在当前开发机执行。

## 现场第 2 步：先核实厂家系统，暂不启动本包控制

现场负责人按已批准的厂家流程准备硬件。本包的硬件 launch **不启动主站、不切换模式、不清故障、不使能驱动器**。

需要明确：
- 真机型号、六轴名称/次序、零位、减速比、网卡、固件与实际部署源码。
- 厂家 automatic_recognition 的最终取值及启动行为。源码默认开启，启动可能执行位置识别/模式或控制字写入。
- 位置控制器是否已运行。厂家 launch 的 load 操作不等于 start。
- 可验证的停止接口与独立硬件看门狗。进程退出、ROS 断网或停止发轨迹不等于机械臂已停止。
- base_pose 定位来源、测量时间与坐标定义；不能把 base_link 与厂家内部旋转了 90° 的 elfin_base 混用。
- 停止 MoveIt Execute、手工轨迹发布者等竞争命令源。ROS1 的 remap 不是访问权限隔离。

先只读查看：

~~~bash
rosparam get /use_sim_time
rosservice call /controller_manager/list_controllers "{}"
rostopic echo -n 1 /joint_states
rostopic echo -n 1 /elfin_arm_controller/state
rostopic hz /joint_states
rostopic hz /elfin_arm_controller/state
rostopic echo -n 1 /base_pose
rostopic echo -n 1 /target_point
~~~

真机要求 /use_sim_time=false；参数不存在时 ROS 默认也是 false。六轴状态需有限数、名称完整、带非零测量时间戳。关节 state 中 desired/actual 必须对应同一采样时刻。所有设备时钟应同步。

base_pose 用 PoseStamped，frame_id=world，姿态为单位四元数，代表 world→elfin_base_link。target_point 用 PointStamped，frame_id=world。本版要求这两类输入持续更新，默认 0.3 秒超时；固定目标也要由上游持续发布新时间戳。**不要用一条 rostopic pub -1 的零时间戳消息启动真机控制。** 上游失联后不得用重打时间戳维持假新鲜数据。

## 现场第 3 步：启动 RViz 与只读监视

确保没有 Gazebo/fake joint_states，也没有厂家固定 world→base 模型的 robot_state_publisher。厂家 elfin5_bringup.launch 中的模型发布器不能与本启动文件重复运行；硬件驱动可继续使用。先检查并人工整理现场启动入口，不直接修改厂家源码。

~~~bash
roslaunch elfin5_qp_controller elfin5_qp_real_robot.launch
~~~

如果定位系统**已经发布** world→elfin_base_link 的动态 TF：

~~~bash
roslaunch elfin5_qp_controller elfin5_qp_real_robot.launch base_tf_mode:=external
~~~

两条命令二选一。pose 模式由 monitor 根据 base_pose 发布动态 TF；发现另一个节点发布该 child frame 会锁定冲突，整理发布者后重启本启动文件。external 模式只监听，并检查外部 TF 与 base_pose 的一致性，仍需提供 base_pose。

本包在 /elfin5_real/robot_description 生成移动基座用模型，仅移除经过结构核验的固定 world_base 与空 world 链接，保留 elfin_base_link→elfin_base 的厂家固定旋转。不覆盖厂家的全局 /robot_description，不修改厂家 xacro。

RViz 默认 Fixed Frame=world，显示：
- 真实关节反馈驱动的机械臂、坐标系；
- 红色目标点、绿色估计实际 TCP、误差连线；
- 有界长度的 world 轨迹、网关状态与末端误差数值。

状态检查：

~~~bash
rostopic echo /elfin5_real/bridge/state
rostopic echo /elfin5_real/monitor/tracking_state
rosrun tf tf_echo world elfin_base_link
rosrun tf tf_echo world elfin_end_link
rostopic info /elfin_arm_controller/command
rostopic info /elfin5_real/candidate_trajectory
~~~

command 的发布者应只有 /elfin5_real/bridge；candidate 的发布者应只有 /elfin5_real/qp_candidate。其他话题发布者和 action goal 发布者会阻止放行。默认 OBSERVE 是正常状态：site_review_complete=false，真实限值未填。

monitor、robot_state_publisher 和 RViz 是观测链，默认启动，但不作为运动准入门禁。它们退出、TF 冲突或误差计算无效会影响显示/数据，不会单独触发网关故障锁存。QP 直接用 base_pose 和关节反馈计算，网关仍检查这些控制输入的格式、时效及时间差。显示或记录失效时，本次实验数据不可据此认定有效；需要结束实验时由操作员显式关闭网关。

显示能动不等于定位准确：移动基座和活动各关节时，逐项比对现场几何、TF、TCP 和关节方向。末端位置由编码器+模型+基座定位估计；绝对物理精度需要外部测量。

## 现场第 4 步：先记录，再启用影子计算

录包由 ROS 自带 rosbag 完成，不在控制回调里写磁盘。另开终端：

~~~bash
source ~/elfin_ws/devel/setup.bash
mkdir -p "$HOME/elfin5_real_logs"
rosbag record -o "$HOME/elfin5_real_logs/elfin5" \
  /joint_states /base_pose /target_point /tf /tf_static \
  /elfin_arm_controller/state /elfin_arm_controller/command \
  /elfin5_hardware/status \
  /elfin5_real/candidate_trajectory /elfin5_real/qp_candidate/state \
  /elfin5_real/bridge/state \
  /elfin5_real/monitor/current_point \
  /elfin5_real/monitor/tracking_state \
  /elfin5_real/monitor/joint_tracking_error
~~~

如果现场改了输入/控制器/硬件状态话题，同步修改录包与绘图命令。录包启动信息必须没有目录或磁盘错误。实验元数据另保存：代码版本、完整参数、定位来源、负载、工况、停止验证记录。可用 rosparam dump 导出本包参数：

~~~bash
rosparam dump "$HOME/elfin5_real_logs/parameters.yaml" /elfin5_real
~~~

在输入正常后，只开启 QP 候选计算：

~~~bash
rosservice call /elfin5_real/qp_candidate/enable "data: true"
rostopic echo /elfin5_real/qp_candidate/state
rostopic hz /elfin5_real/candidate_trajectory
~~~

此时不要调用 bridge/enable true。网关仍不发实际轨迹，可检查 QP、RViz、时间对齐和记录。QP 原有异常分支可能生成 hold，但也只到候选话题。

## 现场第 5 步：完成现场适配与门禁，再放行

先复制一份**站点配置**到现场配置管理目录（不是复制控制器源码）：

~~~bash
mkdir -p "$HOME/elfin5_site"
cp "$(rospack find elfin5_qp_controller)/config/elfin5_real_robot.yaml" \
  "$HOME/elfin5_site/real_robot.yaml"
~~~

在这份配置中填写并审查：
- bridge.position_lower / position_upper：各轴弧度限值，必须在 URDF 限值内。
- bridge.velocity_limits / acceleration_limits：各轴最大速度、加速度。
- bridge.max_endpoint_step：每轴候选端点相对反馈和参考的最大偏移，单位 rad。
- 六轴顺序与 joint_names 一致；空值、NaN、非正速度/加速度禁止放行。
- 硬件 ID、状态话题、停止服务、控制器名、各输入源和超时。
- qp_candidate 内的速度/加速度限制及倍率与站点限值一致；影子计算先验证。不要通过放大网关阈值掩盖轨迹不连续。
- 2026-10-13 用户确认目前只有源码，没有实测频率、最大延迟或厂家停止/看门狗说明。现有采样/时效阈值原样保留，尚不是经过验证的真机参数；不能通过增大阈值消除阻断。需补充的数据及检查分类见 docs/HARDWARE_IMPLEMENTATION_STATUS.md 的简化复核记录。

**站点必须提供的两个接口（本仓库没有厂家专用适配器）：**

| 接口 | 契约 |
|---|---|
| 默认 /elfin5_hardware/status，DiagnosticArray | 持续、非锁存，非零递增时间戳；恰好一个 name=elfin5_hardware，hardware_id 与配置一致。只有现场确认使能、位置模式、无故障、急停/看门狗状态及执行权限都满足时才发 level=OK。未知或任一不满足发 WARN/ERROR/STALE。 |
| 默认 /elfin5_site/stop，std_srvs/Trigger | 请求经现场验证的停止动作；返回 success 的语义由站点明确并验证。ROS 服务回执不自动代表机械臂静止，独立看门狗/物理停止仍必要。 |

这些名字是**本包定义的适配契约，不是已核实存在的厂家服务名**。当前材料无法确定现场监督接口与停止动作，因此没有猜测性地调用厂家使能/刹车/清故障接口。未接好这两项只能只读和影子计算；禁止用固定 OK 假状态或空 stop 服务绕过检查。

网关还要求控制器 running、position interface 占用正确六轴、有效 QP 状态、控制输入时间对齐、无竞争命令源。QP_DEGRADED 也会阻止/关闭实际放行。

完成本站硬件、插值响应、限值、停止/断网看门狗验证后，才把站点配置 site_review_complete 改为 true。这是工程审查记录开关，不是安全认证或自动验证的替代品。配置在启动时读取，修改文件后需关闭网关、完成停止，再重启本包：

~~~bash
roslaunch elfin5_qp_controller elfin5_qp_real_robot.launch \
  hardware_config:="$HOME/elfin5_site/real_robot.yaml" base_tf_mode:=pose
~~~

按定位实际情况选择 pose/external。重新检查输入，调用 qp_candidate/enable true 进入影子计算。确认 /elfin5_real/bridge/state 为 READY 后，在现场许可的低速、小范围、可达目标下：

~~~bash
rosservice call /elfin5_real/bridge/enable "data: true"
~~~

只有服务返回 success=true 且状态变为 ACTIVE 才开始放行。首次实验先静止基座，再逐级尝试平移、转动和组合运动。默认没有世界位置保持前馈：V2 在到点容差内保持关节角，基座继续移动仍会影响 TCP；通过 E(t) 评估，不宣称算法已消除移动基座误差。

## 现场第 6 步：关闭、故障处理与退出

~~~bash
rosservice call /elfin5_real/bridge/enable "data: false"
rosservice call /elfin5_real/qp_candidate/enable "data: false"
~~~

正常关闭 ACTIVE 网关会先禁止继续发轨迹，回到 OBSERVE 并请求站点 stop；请求未完成时不能再次放行。停止服务成功后可回到 READY，但不会自动恢复 ACTIVE，正常关闭不需要 reset。确认现场实际停止后，才退出 launch 和录包。Ctrl+C、杀节点、断网不能替代经验证的停止流程。

ACTIVE 中控制输入失效、控制器/硬件状态异常、QP 降级、非法候选或候选超时都会关闭放行并进入 FAULT_LATCHED。数据恢复不会自动恢复运动。处理原因并确认实际停止后：

~~~bash
rosservice call /elfin5_real/bridge/reset "{}"
~~~

reset 只回到观察/就绪，不使能机械臂。重新放行仍需 bridge/enable true。遇到 STOP_SERVICE_FAILED，reset 明确拒绝：先按现场流程处理停止问题，再人工调用 bridge/enable false 重试一次停止。服务成功回执后仍保持 FAULT_LATCHED，确认机械臂实际静止后再 reset。不会自动重试、自动清除厂家故障或恢复运动；服务 success 仍不能代替物理静止确认。首次故障及停止结果保留到人工复位。

## 现场第 7 步：生成误差曲线

停止录包后，在已安装 ROS Melodic 和 matplotlib 的终端运行，将文件名换成真实记录；output 必须是不存在的新目录：

~~~bash
rosrun elfin5_qp_controller plot_real_robot_error.py \
  "$HOME/elfin5_real_logs/elfin5_实际时间.bag" \
  --output "$HOME/elfin5_real_logs/analysis_001"
~~~

只统计 ACTIVE 阶段可加 --active-only。确认容差后可加 --tcp-tolerance-mm 数值，以及 --joint-tolerance-deg 六个逐轴数值；没有填就不画假定阈值。

输出：
- tcp_error.png：末端 world 位置误差模长 E(t)，单位 mm，不画 XYZ 分量。
- joint_tracking_error.png：J1～J6 六子图，有符号 q_desired−q_actual，单位 °。
- tcp_error.csv、joint_error.csv：原始测量/接收时间、有效误差，以及六轴期望/实际位置和控制器原始 error，内部角度单位 rad。
- events.csv：状态变化、目标变化、基座运动提示。图上最多标记前 50 个事件；完整事件保存在 CSV。
- statistics.json：样本 RMSE、绝对值 P95、最大绝对值、有效覆盖率、给定阈值下的超限时长估计。

各关节期望值取 JTC 当前插值参考，与**同一 state 消息**中的 actual 配对，不拿未来 QP 端点相减。不自动折叠到 ±π；六个有限位关节要求一致零位和角度分支。低频 state 不能证明高频伺服峰值不存在。

末端计算使用关节采样时刻附近的基座位姿（配置限差内最近样本）和不晚于该时刻的目标，不插值伪造更高频数据。监视输出保留参与计算的各时间戳；异常/缺失区间断开曲线，不填零。图的横轴统一为录包接收时间相对起点，测量时间单独保存在 CSV；网络延迟会影响横轴精度。

## 文件分配与实现边界

| 位置 | 职责 |
|---|---|
| src_gazebo/，原 include 头文件 | 原 V2 算法，未改 |
| src/real_robot_bridge_node.cpp | 状态机、输入转发、真实轨迹放行与停止请求 |
| src/real_robot_command_gate.cpp + include/.../real_robot_command_gate.h | 独立可测试的命令形状、时效、限值与保守插值包络检查 |
| src/real_robot_monitor_node.cpp | 真实状态 FK、动态基座 TF、RViz 标记、两类误差 |
| config/elfin5_real_robot.yaml、.rviz | 硬件默认配置与 RViz |
| launch/elfin5_qp_real_robot.launch | 独立硬件接入入口 |
| scripts/prepare_real_robot_description.py | 无损保留内部模型的基座根 URDF 生成 |
| scripts/plot_real_robot_error.py | rosbag→两类误差图、CSV、统计 |
| test/test_real_robot_* | 网关、隔离接线、模型及误差检查 |
| docs/HARDWARE_CONTROL_PLAN.md | 设计依据；当前实现差异见实现检查记录 |

网关的三次 Hermite 包络检查使用**观测到的**控制器参考位置/速度，属于保守候选筛查；异步 state 与实际下一段插值并不完全等同，不能替代下层限值和已验证的停止保护。候选与输入没有统一周期 ID，QP 诊断也不与轨迹原子配对。ROS master 查询存在时间窗口，不能防止恶意或瞬时竞争命令；独立硬件保护和部署管理仍是现场条件。

本次仅修改原 CMakeLists.txt、package.xml、README.md 和硬件方案文档，其他原文件保持。原 Gazebo 测试脚本含运动逻辑，不用作真机数据记录。

<details>
<summary>原有 Gazebo 说明（仅仿真，不能照此启动真机）</summary>


# Elfin5 Cartesian QP Controller V1

面向 ROS Melodic 与 Gazebo 的 Elfin5 末端位置控制器。控制器接收机械臂基座的
绝对位姿和末端目标点的绝对坐标，在不约束末端姿态的前提下，通过在线二次规划
生成满足关节位置、速度和加速度限制的关节命令，并检测、规避运动学奇异区域。

> 当前阶段边界：目标是让末端安全到达给定位置。这里的“QP”是每个控制周期的
> 局部速度控制，不是MoveIt全局路径规划。本版本不包含环境碰撞规划、末端姿态
> 控制、夹爪命令或抓取执行。

> V1 范围：Gazebo 仿真验证。当前 Windows 开发环境不具备 ROS Melodic，因此
> 本包的最终编译与运行验证必须在 Ubuntu 18.04 + ROS Melodic 环境完成。
> 当前真实验证状态统一记录在 [docs/TEST_RESULTS.md](docs/TEST_RESULTS.md)，
> 未登记为PASS的项目均视为尚未验证。

## 1. 设计目标

- 只支持 `elfin5`，规划组固定默认为 `elfin_arm`；
- 主控台通过 `geometry_msgs/PoseStamped` 提供基座绝对位姿；
- 视觉或主控台通过 `geometry_msgs/PointStamped` 提供末端绝对目标点；
- 仅控制末端位置，末端姿态作为冗余自由度；
- 使用凸 QP 同时满足关节位置、速度与加速度约束；
- 通过平移雅可比最小奇异值检测位置任务的奇异性；
- 通过自适应阻尼、奇异值梯度和线性半空间约束进行规避；
- 直接复用现有 Gazebo `elfin_arm_controller`，不修改 EtherCAT 实机代码。

本版本把外部基座接口明确绑定到参数 `base_link`，默认值为
`elfin_base_link`。代码会把MoveIt返回的Jacobian从规划组参考链接
`elfin_base`旋转到 `elfin_base_link`，保证末端位置误差与Jacobian使用同一组坐标轴。

## 2. 软件结构

```text
视觉 / 主控台
  ├── base_pose (PoseStamped)
  └── target_point (PointStamped)
              │
              ▼
QPCartesianController
  ├── 绝对坐标到 base 坐标转换
  ├── MoveIt FK 与 3x6 平移雅可比
  ├── 关节限位和速度/加速度约束
  ├── SVD 奇异检测与梯度规避
  └── BoxQPSolver
              │
              ▼
/elfin_arm_controller/command
              │
              ▼
Gazebo JointTrajectoryController
```

主要文件：

```text
include/elfin5_qp_controller/box_qp_solver.h
    独立、无 ROS 依赖的凸 QP 接口。

src_gazebo/box_qp_solver.cpp
    投影梯度 QP；支持盒约束和一个奇异规避半空间。

include/elfin5_qp_controller/qp_cartesian_controller.h
src_gazebo/qp_cartesian_controller.cpp
    ROS 接口、运动学、约束生成、奇异规避和轨迹输出。

config/elfin5_qp_controller.yaml
    控制频率、速度、容差、关节约束和奇异性阈值。

launch/elfin5_qp_gazebo.launch
    一次启动 Elfin5 Gazebo、MoveIt 和 QP 控制器。

docs/interfaces.md
    完整 ROS 接口和命令行测试方法。

docs/PROJECT_STATE.md
    只记录当前已经存在的代码事实、数据链路和待办事项。

docs/DEV_LOG.md
    记录每次代码修改、原因、风险和未验证部分。

docs/TEST_RESULTS.md
    只记录真实执行过的编译、测试和Gazebo实验结果。

scripts/run_qp_test_suite.py
config/qp_test_suite.yaml
docs/AUTOMATED_TESTS.md
    九轮Gazebo闭环、约束、奇异处理和输入超时自动化测试。
```

新接手开发者建议按以下顺序阅读：

1. `docs/PROJECT_STATE.md`：先确认当前范围、已实现代码和未验证项；
2. `docs/interfaces.md`：确认ROS输入、输出、坐标系和状态含义；
3. `include/elfin5_qp_controller/qp_cartesian_controller.h`：查看类的职责划分；
4. `src_gazebo/qp_cartesian_controller.cpp`：按输入检查、运动学、奇异处理、QP、命令发布顺序阅读；
5. `src_gazebo/box_qp_solver.cpp`：理解 `success` 与 `converged` 的区别；
6. `docs/DEV_LOG.md`：了解最近修改及其原因；
7. `docs/TEST_RESULTS.md`：只从这里确认哪些行为已经被真实验证。

首次实验不要直接测试零位奇异恢复。应先保持 `enable=false`，确认
`current_point`、`min_singular_value`、控制器状态和底层控制器接口正常，再把目标
设置为当前末端附近的小位移。完整门禁步骤和故障分流见
[docs/TEST_RESULTS.md](docs/TEST_RESULTS.md)。
实验中必须保证运动指令来源唯一：MoveIt只在QP禁用时用于设置初始姿态，
QP启用后不再执行MoveIt轨迹或手工关节命令。
`enable=true` 会在三类输入完整、新鲜且坐标系正确时才被接受。

## 3. QP 模型

设当前关节为 `q`，末端位置误差为 `e`，平移雅可比为 `Jv`：

```text
v_des = saturate(Kp * e, maximum_cartesian_speed)
```

每个控制周期求解：

```text
min  0.5 * qdot' * (Jv'Jv + lambda*I) * qdot
     + (-Jv'v_des - lambda*qdot_preferred)' * qdot
```

其中 `qdot_preferred` 同时包含关节居中项和奇异值增大方向。QP 约束为：

```text
-qdot_max <= qdot <= qdot_max
q_min + margin <= q + qdot*dt <= q_max - margin
-qddot_max*dt <= qdot - qdot_last <= qddot_max*dt
grad(sigma_min)' * qdot >= k * (sigma_safe - sigma_min)  # 接近奇异区时
```

QP 使用 Eigen 实现的投影梯度法，不增加 OSQP、qpOASES 等系统依赖。问题只有
6 个变量，适合 V1 的 50 Hz Gazebo 在线验证。后续实机版本如需严格确定性和更多
碰撞约束，可以在保持上层接口不变的情况下替换求解器。

## 4. 奇异性策略

姿态不受控制，因此检测的是平移雅可比 `Jv`，而不是完整的 6x6 几何雅可比。

- `sigma_min > warning`：使用较小的常规阻尼；
- `stop < sigma_min <= warning`：逐渐增加阻尼，并激活奇异值梯度规避；
- `sigma_min <= stop`：优先沿奇异值增大方向恢复；中央差分在精确奇异点退化
  时，比较正负单侧扰动并选择能够提高最小奇异值的方向；如果没有找到恢复方向
  或恢复约束不可行，发布保持当前位置命令并输出明确异常状态；
- 奇异规避约束与关节约束冲突：停止并输出 `QP_INFEASIBLE`。

QP结果区分两种情况：

- `success=true, converged=true`：结果满足约束，投影梯度达到容差；
- `success=true, converged=false`：结果满足安全约束，但达到最大迭代次数，控制器
  继续使用该可行速度并发布 `QP_DEGRADED`，便于后续根据Gazebo数据判断是否需要
  更换求解器或增加迭代次数。

阈值只是 V1 初始值，应结合 Elfin5 Gazebo 数据曲线调参。

## 5. 编译环境

目标环境：

```text
Ubuntu 18.04
ROS Melodic
catkin workspace
MoveIt 1 for ROS Melodic
Gazebo 9
```

安装常见依赖：

```bash
sudo apt update
sudo apt install ros-melodic-moveit ros-melodic-gazebo-ros-control
```

将整个 `elfin_robot-melodic` 放入 catkin 工作空间的 `src` 中：

```bash
mkdir -p ~/elfin_ws/src
cp -a elfin_robot-melodic ~/elfin_ws/src/
cd ~/elfin_ws
rosdep install --from-paths src --ignore-src -r -y
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

也可以只编译新增包及其依赖：

```bash
catkin_make --pkg elfin5_qp_controller
```

运行单元测试：

```bash
catkin_make run_tests_elfin5_qp_controller
catkin_test_results
```

## 6. Gazebo 启动

一次启动完整验证环境：

```bash
roslaunch elfin5_qp_controller elfin5_qp_gazebo.launch
```

无 RViz：

```bash
roslaunch elfin5_qp_controller elfin5_qp_gazebo.launch display:=false
```

如果 Gazebo 和 MoveIt 已经启动，只启动控制器：

```bash
roslaunch elfin5_qp_controller elfin5_qp_controller.launch
```

## 7. 第一次运动测试

先确认关节控制器正常：

```bash
rostopic echo -n 1 /joint_states
rostopic info /elfin_arm_controller/command
```

发布基座绝对位姿。当前上游Gazebo启动文件通过 `-z 0.1` 生成模型，因此这里的
`elfin_base_link` 高度使用0.1米：

```bash
rostopic pub -r 10 /elfin5_qp_controller/base_pose geometry_msgs/PoseStamped \
  "header: {frame_id: 'world'}
pose:
  position: {x: 0.0, y: 0.0, z: 0.1}
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}"
```

发布目标点。以下坐标只是接口示例，正式测试应先在 RViz/Gazebo 中确认可达：

```bash
rostopic pub -r 10 /elfin5_qp_controller/target_point geometry_msgs/PointStamped \
  "header: {frame_id: 'world'}
point: {x: 0.35, y: 0.10, z: 0.45}"
```

开始控制：

```bash
rosservice call /elfin5_qp_controller/enable "data: true"
```

观察：

```bash
rostopic echo /elfin5_qp_controller/position_error
rostopic echo /elfin5_qp_controller/min_singular_value
rostopic echo /elfin5_qp_controller/state
```

停止：

```bash
rosservice call /elfin5_qp_controller/enable "data: false"
```

完整接口参见 [docs/interfaces.md](docs/interfaces.md)。

## 8. 验收建议

以下项目是后续Gazebo阶段的验收要求，不代表当前已经通过。真实结果必须写入
`docs/TEST_RESULTS.md`：

1. 普通可达点最终误差不大于 `goal_tolerance`；
2. 改变 `base_pose` 后，绝对目标点的转换结果正确；
3. 六个关节位置始终位于 URDF 安全范围内；
4. 关节速度不超过配置值乘 `velocity_limit_scale`；
5. 接近奇异区域时 `min_singular_value` 可见、速度无尖峰；
6. 不可行约束触发 `QP_INFEASIBLE` 并停止运动；
7. 输入超过 `input_timeout` 后触发 `STALE_INPUT`；
8. 更新目标点时没有明显关节位置跳变。

## 9. V1 已知边界

- 尚未加入环境障碍物距离约束；Gazebo V1 聚焦目标到达、关节约束和奇异规避。
- 当前是局部速度伺服，不计算全局无碰撞路径，也不承诺避开环境障碍物。
- 当前不约束末端姿态，不包含夹爪和抓取执行。
- 基座位姿只参与坐标换算，不会在 Gazebo 中移动固定基座模型。
- `data=false` 会先发布一次当前位置保持命令，随后停止QP命令输出；硬实时急停仍
  必须由控制器或驱动层实现。
- 投影梯度求解器支持盒约束和一个线性半空间，不是通用多约束 QP 求解器。
- 当前代码尚未在目标ROS Melodic环境完成本轮修改后的编译和仿真验收。

## 10. 调参顺序

建议按以下顺序调试，避免多个因素互相干扰：

1. 将目标点设置在初始末端附近，确认坐标系方向；
2. 调整 `position_gain` 与 `maximum_cartesian_speed`；
3. 验证位置、速度、加速度限制；
4. 记录正常姿态下的 `min_singular_value` 范围；
5. 再调整 warning、safe、stop 三个奇异性阈值；
6. 最后调整 `singularity_avoidance_gain` 和 `joint_centering_gain`。

</details>
