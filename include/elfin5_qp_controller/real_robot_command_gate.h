/*
 * real_robot_command_gate.h
 * 硬件命令校验核心的公共接口。
 * 职责：描述现场限值、候选轨迹和纯检查函数；不依赖 ROS，不访问 EtherCAT。
 * 数据约定：names 定义六轴顺序；位置/步长用 rad，速度用 rad/s，加速度用 rad/s²，时间用 s。
 * 数据流：ROS 网关提取消息字段 → checkCommand 审查 → 网关决定是否原样转发。
 * 返回约定：检查失败返回 false，reason 给出可记录的原因码；不会修正或限幅输入。
 * 使用边界：这里的三次曲线包络基于观测参考状态，不代表真实下层插值已经被严格证明。
 * 阅读顺序：HardwareLimits → HardwareCommand → orderedPositions/fresh → checkCommand。
 */

#pragma once
#include <string>
#include <vector>

namespace elfin5_qp_controller {
// 现场人工核验后的限制集合；默认空数组有意使 valid() 失败，避免拿示例参数直接放行。
struct HardwareLimits {
  std::vector<std::string> names;  // 标准六轴名称/顺序，不能仅凭数组索引猜测关节。
  std::vector<double> lower, upper, velocity, acceleration, step;  // 依次为位置下限、上限、速度上限、加速度上限和最大端点偏移。
  double min_horizon = 0.02, max_horizon = 0.15;  // 候选 time_from_start 的允许范围，单位秒；仍需现场核验。
  bool valid(std::string& reason) const;  // 校验配置完整性及数值关系，不查询机械臂。
};
// 按 expected 的关节名称重排 positions；拒绝缺失名称、重复名称、长度不符与非有限值。
// positions 也可传速度数组。失败时 result 可能已部分写入，调用方必须检查返回值。
bool orderedPositions(const std::vector<std::string>& expected,
                      const std::vector<std::string>& names,
                      const std::vector<double>& positions,
                      std::vector<double>& result);
// 双时间条件：now/stamp 使用同一 ROS 时间域，wall_age 是本机接收后的墙钟间隔。
// future_tolerance 只容许很小的时钟误差；stamp=0 不作为有效测量。
bool fresh(double now, double stamp, double wall_age, double timeout,
           double future_tolerance = 0.01);
// 从 JointTrajectory 单点消息提取出的最小候选数据；结构本身不进行消息类型检查。
struct HardwareCommand {
  double stamp = 0.0, horizon = 0.0;  // 轨迹消息生成时间与相对该时间的端点时域。
  std::vector<std::string> names;  // 标准六轴名称/顺序，不能仅凭数组索引猜测关节。
  std::vector<double> positions, velocities;  // 单个轨迹端点的位置和速度，不是当前反馈。
};
// Pure checks; no ROS, no publishing, no QP or trajectory rewriting.
// actual 是真实关节位置；reference/reference_velocity 是下层控制器当时的插值参考。
// epoch 是本次显式放行的时间；previous_stamp 是上次已接受命令的时间，用来拒绝旧代次/重放。
// 所有参考数组都须已按 limits.names 排列。函数只读输入，reason 是唯一输出参数。
bool checkCommand(const HardwareLimits& limits, const HardwareCommand& command,
                  const std::vector<double>& actual,
                  const std::vector<double>& reference,
                  const std::vector<double>& reference_velocity,
                  double now, double epoch, double previous_stamp,
                  double max_age, std::string& reason);
// 位置“安全”仅指有限且位于已配置限位内，不代表碰撞或现场几何已验证。
bool positionsWithinLimits(const HardwareLimits& limits, const std::vector<double>& positions);
// 以最近有效实际反馈为保持目标，端点速度为零，仍通过同一插值包络检查。
// 调用方须先确认反馈新鲜、驱动可用；本函数不连接硬件、不证明已经停稳。
bool makeHoldCommand(const HardwareLimits& limits, const std::vector<double>& safe_pose,
                     const std::vector<double>& reference,
                     const std::vector<double>& reference_velocity,
                     double now, double horizon, HardwareCommand& hold, std::string& reason);
std::string commandReasonChinese(const std::string& reason);
}  // namespace elfin5_qp_controller
