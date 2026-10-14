# Elfin5 QP 控制器

面向机械臂控制算法研究的 ROS 包，包含当前 V2 QP 控制器、Gazebo 仿真入口，以及通过厂家 EtherCAT 驱动和 JTC 接入真机的接口。

末端只跟踪 XYZ，允许姿态变化。`world` 是固定坐标系；基座可以移动，输入基座位姿和目标点均在 `world` 中表达。

控制链：真实反馈 → QP 候选轨迹 → 硬件命令网关 → 厂家位置 JTC → RobotHW → EtherCAT。真机启动文件默认打开 RViz，支持末端位置误差范数和六个关节过程跟踪误差的记录、绘图。

## 文档入口

- [当前接口](docs/interfaces.md)：输入输出、服务、诊断、状态机及配置约定。
- [基本使用](docs/usage.md)：编译、现场观察、影子计算、人工放行与停止、误差绘图。

## 环境与依赖

目标环境为 Ubuntu 18.04、ROS Melodic、Python 2.7。将本仓库与设备对应的厂家 ROS 包放入同一 catkin 工作空间的 `src`；ROS 包名为 `elfin5_qp_controller`，同一工作空间只保留一份。

厂家源码参考：[huayan-robotics/elfin_robot 的 melodic 分支](https://github.com/huayan-robotics/elfin_robot/tree/melodic)。设备专属标定、驱动配置及实际部署版本需现场核对。

## 文件分工

| 目录 | 用途 |
|---|---|
| `src_gazebo/`、`include/` | 现有 QP 求解器、笛卡尔控制器及接口头文件 |
| `src/` | 真机命令网关、命令检查、只读监视 |
| `launch/`、`config/` | 仿真/真机入口、算法和站点参数、RViz 配置 |
| `scripts/` | 模型准备、误差绘图和仿真测试工具 |
| `test/` | 求解器、命令检查、模型/误差及 ROS 接线测试 |
| `docs/` | 当前接口与基本使用说明 |

## 当前使用边界

默认配置用于观察和影子计算：真实关节限值为空、现场审查开关关闭，不能直接放行运动。现场硬件状态适配器和停止服务需另行接入；本包不自动使能驱动器或清除厂家故障。

已有 Python 离线检查通过；完整 ROS 构建、ROS 隔离测试和真机验证尚未完成。通信周期、最大延迟及厂家停止/看门狗资料尚未取得，现有超时参数不能作为真机验收依据。具体操作及待核实项见[基本使用](docs/usage.md)。
