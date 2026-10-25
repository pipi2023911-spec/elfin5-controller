# Elfin5关节限值参数核对

## TASK

根据用户提供的手册位置/速度截图保留10%余量；解释并设置未公布额定值的加速度软件初值。先核对现有QP、真机候选配置与用户提供的旧厂家源码，最小修改配置和限值合并逻辑，不改求解器、网关状态流程或厂家文件。

涉及`config/elfin5_qp_controller.yaml`、`config/elfin5_real_robot.yaml`、`src_gazebo/qp_cartesian_controller.cpp`、`test/test_controller_limits.py`、CMake测试注册及README/doc说明。验收包括六轴换算、配置不扩大模型、每周期命令变化量、非法参数/冲突拒绝和既有离线回归。

## TASK_RESULT

实际配置已设置手册位置/速度的90%，与MoveIt模型取交集；启动日志输出实际生效值。基础软件加速度1.0 rad/s²，真机候选0.25 rad/s²，均未标为厂家额定值。旧MoveIt文件的12.56只作为旧软件参数记录，没有当成手册额定加速度。

| 已执行检查 | 结果 |
|---|---|
| 六轴手册90%及角度/弧度换算 | PASS |
| 实际C++限值合并片段和`buildVelocityBounds`方法，真实Eigen，ROS参数/日志桩 | 64项断言PASS |
| `test_real_robot_tf_and_error.py -v` | 14项既有Python离线测试PASS |
| 初次Windows Zig + Eigen3.3.4编译 | FAIL；旧Eigen模板解析兼容问题，保留原日志 |
| Zig测试路径添加`-fdelayed-template-parsing`后 | 编译和上述64项运行PASS，第三方源码未修改 |
| 完整ROS编译、catkin测试、Gazebo R1–R3 | 未验证 |
| 真机通信/运动/停止及实际加速度 | 未验证，未启动设备 |

64项包含宽模型下使用手册90%上限、旧模型不能被放宽、20 ms下±0.02 rad/s命令窗口、候选±0.005 rad/s窗口、非零历史速度、位置/减速度约束无交集，以及缺失/错误维度/NaN/零加速度拒绝。这些是离线逻辑测试，不是完整ROS节点构建或机械臂实验。

## 模型差异与当前边界

手册90%位置为±[324,121.5,137.7,324,162,324]°，速度为[162,162,162,162,180,180]°/s。用户提供的旧URDF位置更严，六轴速度1.57 rad/s；因此不能声称运行中已经使用全部手册范围。

旧模型与新配置的交集位置为±[3.14,2.120575041,2.403318380,3.14,2.56,3.14] rad，基础速度仍为1.57 rad/s。实际部署可能使用其他模型，须看启动日志。逐轴值与只读核对文件哈希见[limits.json](limits.json)；截图见[原图](../../elfin_joint_limits_reference.png)。没有修改外部URDF、驱动或MoveIt文件。

## 运行命令和原始日志

Ubuntu复现新增离线测试：

```bash
python3 test/test_controller_limits.py --eigen /usr/include/eigen3
python test/test_real_robot_tf_and_error.py -v
```

本次Windows从仓库目录使用已有Python环境和编译器：

```powershell
python test/test_controller_limits.py --eigen ../eigen-compat/eigen-git-mirror-3.3.4 --zig ../build-tools/ziglang/zig.exe
python test/test_real_robot_tf_and_error.py -v
```

`test_logs.zip`保存初次编译失败、兼容后通过、14项回归原始日志及证据打包脚本。初次失败日志对应添加兼容选项之前的测试脚本；当前脚本Zig路径已自动带上兼容选项，Ubuntu/GCC没有该项。通过日志包含本机工具链标准库警告，不宣称无警告编译。

运行时父提交、五个实际源码/配置摘要及归档SHA256见[verification.json](verification.json)。父提交不代表新增代码；源码哈希用于区分本次运行状态。截图不含完整手册型号/版本，不推断为已核实现场硬件规格。

## 问题与下一步

1.0和0.25 rad/s²仅为软件初值。QP约束的是速度命令差分，不等于JTC插值或实际关节加速度；不包含完整制动距离规划。完整接口与解释见[关节限值说明](../../joint_limits.md)。

下一步最小验证是Ubuntu完整编译并重跑Gazebo R1–R3，保存实际生效限制、关节命令/反馈、JTC参考和误差；旧轨迹结论不能直接沿用。真机保持原有人工放行和站点授权要求，本次没有自动使能、改变站点审查标志或填入未经核实的现场限值。
