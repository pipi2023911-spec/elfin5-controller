# BUG-HORIZON-001：轨迹覆盖缺口与固定140ms适配

## TASK

保留旧版50ms轨迹可能在厂家下一次接收前用完、从而产生端点等待/启停的Debug材料；在不修改厂家源码的前提下，将QP输出改为固定140ms，并提交GitHub。已验证基础为`5223f02`源码与单关节时序模型，未运行真实机械臂。

## 问题与原因

旧配置`control_rate=50`、`command_horizon_factor=2.5`，正常dt=20ms时生成H=50ms单点轨迹。厂家硬件节点主线程标称每100ms调用spinOnce，JTC新命令处理与1kHz插值是两件事。旧端点在下一次接收前结束，JTC会保持端点并将期望速度置零，随后收到新点再运动。

例如80ms生成的旧点在100ms被接收，端点时间130ms；下一次约200ms才处理新点，中间可能等待约70ms。标称100ms不是最坏接收间隔，实际还受服务阻塞、调度和通信影响。

**证据等级：源码确认覆盖缺口；理想模型复现启停；真机抖动原因及修复效果未验证。** 本记录不把模型零速比例、峰值差分加速度或停发位移当作机械臂实测数据。

## 旧版保留与复现

- 基线提交：`5223f02b742a70caa0febe1bbaeb5ae2dc46ca90`。
- Git标签：`debug/horizon-50ms-20261027`，指向同一基线，保留完整工程历史。
- [baseline_50ms.zip](baseline_50ms.zip)：从该Git提交提取的配置、控制器头/源文件、当时的时域说明，共4个文件；不是当前运行文件。
- [baseline_manifest.json](baseline_manifest.json)：归档及成员SHA256，同时记录本轮不得修改的求解器、网关、停止适配器、站点配置与厂家关键文件摘要。

只读查看旧配置：

```bash
git show debug/horizon-50ms-20261027:config/elfin5_qp_controller.yaml
git show debug/horizon-50ms-20261027:src_gazebo/qp_cartesian_controller.cpp
```

离线重现覆盖缺口（开发机Python3，需Matplotlib；输出使用新目录）：

```bash
python3 test/test_command_horizon.py -v
python3 scripts/validate_command_horizon.py --output /tmp/elfin_horizon_01
```

脚本同时比较50/140ms，不启动ROS或设备。若需要旧工程实验，使用隔离目录和仿真环境，不能把基线ZIP直接覆盖当前现场工程。

## 最小修改与不采用的方案

采用`command_horizon: 0.14`秒替换倍率参数；正常运动和QP保持共用固定值。位置端点`q_meas+H*dq`、位置预测约束和消息`time_from_start`使用同一个H；相邻QP速度差限制仍用实际dt。

| 当次dt | 旧factor=2.5 | 倍率7候选 | 当前固定参数 |
|---|---:|---:|---:|
| 10ms | 25ms | 70ms | 140ms |
| 20ms | 50ms | 140ms | 140ms |
| 30ms | 75ms | 210ms | 140ms |

没有提高QP频率，因为保持旧倍率会让时域更短；没有采用倍率7，因为dt抖动会使时域超出当前150ms网关上限。`command_horizon_factor`已移除，旧站点副本需删除该键并迁移为秒制参数。

不修改厂家、数值QP求解器、100步预算、50Hz频率、5mm容差、关节参数和现有网关检查。网关故障保持时域仍100ms，不与QP的140ms保持混淆。默认站点审查仍关闭、限值数组仍为空。

## 实际测试与结果

| 验证 | 实际结果 | 能证明什么 |
|---|---|---|
| 编译实际QP方法片段与实际网关核心 | 144项断言PASS | 参数读取/无效参数拒绝、固定H、运动与保持端点、位置边界用H/加速度边界用dt、网关样例契约 |
| 时域模型单元测试 | 4项PASS | 解析样条、覆盖缺口、延迟及停发条件 |
| 既有模型/误差/接线离线检查 | 14项PASS | 既有纯函数与launch文本契约 |
| 重新运行单关节理想模型 | 21案例完成 | 采样相位、100/200ms回调及停发敏感性 |
| 完整ROS/MoveIt节点构建、Gazebo、EtherCAT、真实停车/运动 | 未验证 | 不得以方法桩/模型PASS代替 |

实际Windows命令（仓库根目录，使用已有Python3.12.14虚拟环境与Zig/Eigen3.3.4）：

```powershell
python test/test_qp_command_horizon.py --zig ../build-tools/ziglang/zig.exe --eigen ../eigen-compat/eigen-git-mirror-3.3.4
python test/test_command_horizon.py -v
python test/test_real_robot_tf_and_error.py -v
python scripts/validate_command_horizon.py --output ../../audit/horizon-fixed140-2026-10-27
```

方法测试提取实际参数加载与校验、发布/保持、速度边界方法，以及运动端点计算语句；链接实际网关纯检查源文件。ROS参数/时间/发布器是桩，MoveIt与完整timer回调未运行。它不是完整节点编译或目标机实时性测试。

固定采样相位、巡航1–4秒的模型对照：

| 条件 | 零速比例 | 峰值速度 / rad/s |
|---|---:|---:|
| 100ms回调、50ms时域 | 68% | 0.167320 |
| 100ms回调、140ms时域 | 0% | 0.073978 |
| 200ms回调、140ms时域 | 39% | 0.120547 |

![单关节模型巡航窗口，非机械臂实测](horizon_comparison.png)

完整21条件见[comparison.csv](comparison.csv)。原始轨迹、脚本版本与日志见[raw_evidence.zip](raw_evidence.zip)，当前源码及归档校验见[verification.json](verification.json)。

## 失败/边界必须保留

1. **旧50ms覆盖失败**：相位组合下出现51%–70%模型零速等待，保留旧源码和原始数据。
2. **140ms并非覆盖所有延迟**：回调200ms时仍有39%零速。不能把“名义巡航不断轨”扩展成全场景保证。
3. **起步与停发边界**：140ms名义案例起步差分峰值约0.756rad/s²；4秒停发案例额外位移约0.008690rad，末端速度截断产生较大差分。这些是理想模型量，不是硬件停距或机械加速度。
4. **本机编译权限**：首次受限执行因WinError5无法启动编译器，未发生实际编译；保留原日志，获准使用已有编译器后144断言通过。不能写成算法失败或偷偷修改生产参数来通过。

旧模型与旧版本不会被新成功结果覆盖。本轮仅修复轨迹时域与控制周期耦合、补足名义接收间隔覆盖；没有证明其他通信、执行或反馈问题已经消失。

## TASK_RESULT与下一步

实际修改为QP配置/成员参数/输出时域、独立方法回归与CMake注册，以及README、接口、使用、测试、时域和Debug文档。检查结果PASS仅限本节明确列出的Windows方法/模型层级。

下一步在ROS Melodic完整构建，核对实际参数为0.14秒；隔离验证接线和stop联动，再以相同目标/姿态/其他参数对比旧版与当前版。记录JTC实际接受间隔、desired/actual、拒绝原因、到达及停发/停止位移。真机仍须完成原有现场授权与停止链核验。
