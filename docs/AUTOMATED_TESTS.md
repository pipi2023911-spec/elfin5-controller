# Elfin5 QP 九轮自动化测试

`scripts/run_qp_test_suite.py` 用于 Gazebo 环境中的闭环集成测试。它自动维持
`base_pose`、调用 MoveIt 设置初始关节姿态、生成相对笛卡尔目标、控制QP使能、
判断结果并保存 CSV、Markdown 和 rosbag。

脚本明确兼容 Ubuntu 18.04、ROS Melodic 自带的 Python 2.7；未使用 Python 3
专属语法。请使用 `python`/`rosrun`，不要强制使用 `python3`。

## 前提

先启动完整环境：

```bash
roslaunch elfin5_qp_controller elfin5_qp_gazebo.launch display:=true
```

不要另外运行手工 `base_pose` 或 `target_point` 发布器，测试脚本会独占这两个输入。
启动测试前不要在RViz中执行轨迹。

## 构建

新增脚本后重新构建并加载工作空间：

```bash
cd ~/catkin_ws
catkin_make --pkg elfin5_qp_controller
source devel/setup.bash
```

## 首次运行

先只检查依赖、控制器、话题和每轮配置，不执行运动：

```bash
rosrun elfin5_qp_controller run_qp_test_suite.py \
  --rounds R1,R2,R3,R4,R5,R6,R7 \
  --dry-run
```

运行普通闭环轮次：

```bash
rosrun elfin5_qp_controller run_qp_test_suite.py \
  --rounds R1,R2,R3,R4
```

脚本默认在每轮开始前要求输入 `RUN R1`、`RUN R2` 等确认，并在首个失败后停止。

运行奇异处理轮次：

```bash
rosrun elfin5_qp_controller run_qp_test_suite.py \
  --rounds R5,R6,R7
```

R8会把机械臂设置到已知精确奇异零位，必须额外确认：

```bash
rosrun elfin5_qp_controller run_qp_test_suite.py \
  --rounds R8 --allow-deep-singularity
```

完全无人值守运行仅建议用于已经验证稳定的Gazebo环境：

```bash
rosrun elfin5_qp_controller run_qp_test_suite.py \
  --rounds R1,R2,R3,R4,R5,R6,R7,R8,R9 \
  --non-interactive --allow-deep-singularity
```

## 结果

默认写入：

```text
docs/test_runs/YYYYMMDD_HHMMSS/
```

包括：

- `summary.md`：总表与每轮详细结果；
- `parameters.yaml`：测试配置和运行时控制器参数；
- `rounds.json`：机器可读的轮次结果；
- `controller_state.csv`：状态、QP迭代次数和残差；
- `position_error.csv`、`singular_value.csv`；
- `joint_states.csv`、`trajectory_commands.csv`；
- `targets.csv`；
- `qp_test.bag`。

如果包安装目录不可写，脚本自动改写到：

```text
~/.ros/elfin5_qp_controller/test_runs/
```

可通过 `--output` 指定其它目录，通过 `--no-bag` 关闭rosbag。

## 安全门禁

- 每轮先关闭QP并等待 `IDLE`；
- 禁用状态下若 `/elfin_arm_controller/command` 仍持续发布，测试直接失败；
- MoveIt执行完成后会比较实际关节角与请求角；
- 实际初始奇异值不在配置区间时，不允许进入该轮；
- 默认任一轮失败就停止；
- R8需要显式 `--allow-deep-singularity`；
- 退出和异常处理中均调用 `enable=false`。

测试配置集中位于 `config/qp_test_suite.yaml`。
