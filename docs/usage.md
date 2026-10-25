# 编译与操作

## 环境准备

目标平台Ubuntu 18.04 / ROS Melodic / Python2.7。使用设备匹配的厂家elfin_robot包、MoveIt、Gazebo和Eigen。此包包名为elfin5_qp_controller，同一工作空间只保留一份。

```bash
source /opt/ros/melodic/setup.bash
cd ~/catkin_ws
catkin_make -DCATKIN_ENABLE_TESTING=ON
source devel/setup.bash
rospack find elfin5_qp_controller
```

厂家模型包和运行库版本必须与设备一致；不要从不明配置复制编码器零位、减速比、网卡、转矩因子。依赖安装应通过已配置的ROS环境/rosdep处理。脚本run_qp_test_suite.py的CSV写入针对Python2.7，不能用Python3语法检查通过代替其运行验证。

## Gazebo基础实验

先按[QP算法验收](qp_solver.md)完成独立数值测试，再运行仿真。求解器改变可能使原先弱效的关节居中偏好明显生效；先保持控制增益与目标不变，分别保存旧/新版本结果，不能假定末端轨迹完全相同。

仅在隔离的仿真工作空间执行，真机EtherCAT节点不启动。

终端一：

```bash
source ~/catkin_ws/devel/setup.bash
roslaunch elfin5_qp_controller elfin5_qp_gazebo.launch
```

该入口调用厂家Gazebo与MoveIt，加载QP；不会启动真机bridge。QP默认未启用，需基座/目标输入。最小实验使用已有自动脚本，它会发布这些输入、设置MoveIt初始关节位姿，再显式启用QP。

终端二：

```bash
source ~/catkin_ws/devel/setup.bash
rosrun elfin5_qp_controller run_qp_test_suite.py --rounds R1 --output "$HOME/elfin5_sim/R1_01"
```

按提示输入RUN R1才执行该轮。先确认R1可重复，再逐步选择R2至R9；R8深奇异轮默认需要额外选项，不作为首轮实验。不要将此脚本连接到真机master，它包含MoveIt运动及QP启用操作。

原始输出含qp_test.bag、参数、joint_states.csv、trajectory_commands.csv、position_error.csv、singular_value.csv、controller_state.csv、targets.csv、rounds.json和summary.md。CSV误差为范数，XYZ分量需由bag中同系且按时间对齐的current_point/target恢复，脚本目前不自动给出XYZ分量图或QP求解耗时。

要记录同一时刻的关节过程误差，另录JTC state：

```bash
rosbag record -O "$HOME/elfin5_sim/jtc_R1_01.bag" /elfin_arm_controller/state /joint_states
```

每次固定代码版本、初始姿态、目标偏移和配置，至少分别保存正常、约束/奇异及失败工况。通过标准见[测试文档](testing.md)。

## 真机前的现场核对

厂家elfin5_bringup.launch只加载URDF和robot_state_publisher；elfin_ros_control.launch才启动硬件接口，并且只load手臂控制器，不保证running。厂家驱动automatic_recognition默认true，硬件启动可能识别位置、清故障/撤销使能，不能当成被动观察。硬件启动、识别、驱动使能和controller切换由现场授权人员按已确认流程执行，本包不自动执行。

先读取设备实际接口：

```bash
rostopic echo -n 1 /joint_states
rostopic echo -n 1 /elfin_arm_controller/state
rostopic echo -n 1 /elfin_ros_control/elfin/enable_state
rostopic echo -n 1 /elfin_ros_control/elfin/fault_state
rosservice call /controller_manager/list_controllers "{}"
rostopic hz /joint_states
rostopic hz /elfin_arm_controller/state
```

核对六轴名称/符号、模型、站点限位、负载、实际停止保持接口及总线中断行为；消除MoveIt/GUI/其他程序的竞争命令源。Bool状态新鲜不等于PDO新鲜；必须明确实际链路失效和执行层看门狗行为。

站点提供std_srvs/Trigger停止保持服务，默认/elfin5_site/stop_hold。必须先独立验证正常运动、反馈失效与驱动异常下的停止/保持行为；空服务、固定OK、单纯类型重映射不能满足契约。详见[接口](interfaces.md)。

当前开发环境是Windows，可先按[stop适配器](stop_adapter.md)运行不连接设备的离线测试。可选适配器有单独launch，真机主入口不会自动启动；本轮没有执行该ROS launch。不能仅以服务存在或桩测试通过设置现场审查标志。时域模型入口及未执行的ROS验证见[时域说明](horizon_validation.md)。

## 观察与影子计算

复制站点配置：

```bash
mkdir -p "$HOME/elfin5_site"
cp "$(rospack find elfin5_qp_controller)/config/elfin5_real_robot.yaml" "$HOME/elfin5_site/real_robot.yaml"
```

填写已核实限值与接口。保持site_review_complete=false，未调用bridge/enable true时没有实际命令输出。参数不热更新。

在厂家反馈与模型入口已按现场流程运行后：

```bash
roslaunch elfin5_qp_controller elfin5_qp_real_robot.launch hardware_config:="$HOME/elfin5_site/real_robot.yaml"
```

定位系统持续提供/base_pose，主控提供/target_point，均为absolute_world。pose模式由monitor发布absolute_world→world；若定位系统已发布这条动态TF，用base_tf_mode:=external，仍提供一致的base_pose。厂家发布模型内部TF，不额外启动重复robot_state_publisher。

先记录：

```bash
mkdir -p "$HOME/elfin5_real_logs"
rosbag record -o "$HOME/elfin5_real_logs/elfin5" \
  /joint_states /base_pose /target_point /tf /tf_static \
  /elfin_arm_controller/state /elfin_arm_controller/command \
  /elfin_ros_control/elfin/enable_state /elfin_ros_control/elfin/fault_state \
  /elfin5_real/candidate_trajectory /elfin5_real/qp_candidate/state \
  /elfin5_real/bridge/state /elfin5_real/monitor/current_point \
  /elfin5_real/monitor/tracking_state /elfin5_real/monitor/joint_tracking_error
```

另终端：

```bash
rosparam dump "$HOME/elfin5_real_logs/parameters.yaml" /elfin5_real
rosservice call /elfin5_real/qp_candidate/enable "data: true"
rostopic echo /elfin5_real/qp_candidate/state
rostopic hz /elfin5_real/candidate_trajectory
```

此时只启用候选计算，bridge仍关闭。确认实际command话题没有来自本包的输出。候选即使是保持命令也不能穿过未放行网关。

## 受控真机闭环

只有现场权限、停止设施、保持路径、限位及所有前置核验完成后，才将站点site_review_complete设为true并重启本包。该标志不执行验证，也不使能驱动。

先重新开启影子计算，检查候选和反馈。首轮固定基座、低速、小范围目标，由现场人员执行：

```bash
rosservice call /elfin5_real/bridge/enable "data: true"
rostopic echo /elfin5_real/bridge/state
```

检查服务success和ACTIVE；READY只是输入条件满足的显示，控制器/停止服务还在enable时核对。不要把影子输出、EtherCAT连通或RViz模型运动称为真机闭环。

### 关闭与保持

```bash
rosservice call /elfin5_real/bridge/enable "data: false"
rosservice call /elfin5_real/qp_candidate/enable "data: false"
```

正常关闭与故障都会进入FAULT_LATCHED：候选阻断，条件允许时一次保持请求，并调用站点停止保持服务。等待服务结果，同时查看关节位置/速度与现场实际静止。不要用Ctrl+C或停发命令替代停止。

保持是最近有效反馈姿态，不是上一个候选目标；反馈已经失效或驱动报告故障时，网关不下发历史位置回拉，交给站点已验证的停止保持路径。故障注入先在隔离测试和Gazebo执行，禁止为测试而直接拔真机总线或伪造硬件状态。

确认停止后再处理原因。服务失败时先用现场独立措施处置，修复接口后明确再次调用bridge/enable false重试；不自动重试。服务成功应答后：

```bash
rosservice call /elfin5_real/bridge/reset "{}"
```

reset只关闭地清除锁存。重新运动需重新影子检查和人工enable true，数据恢复不能自动恢复运动。退出前先执行停止保持流程，核实执行层看门狗；bridge析构不提供已验证的物理停车保证。

## 数据分析

停止录包后，使用新输出目录：

```bash
rosrun elfin5_qp_controller plot_real_robot_error.py "$HOME/elfin5_real_logs/实际文件.bag" --output "$HOME/elfin5_real_logs/analysis_01"
```

可加--active-only只统计ACTIVE片段。输出TCP误差范数、六轴参考/反馈/误差曲线、统计及事件；样本RMSE不是时间积分RMSE。缺失/无效段断线，不补零。TCP来自FK与定位估计，不等同外部测量精度。

现有图脚本不自动生成XYZ分量误差或QP耗时。实验报告须明确未实现项；未来候选端点与当前反馈不能直接相减作为过程跟踪误差。每轮保留原始bag、参数、版本、图、失败原因和用户自主修改的核心逻辑。
