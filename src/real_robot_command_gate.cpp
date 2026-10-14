/*
 * real_robot_command_gate.cpp
 * 硬件候选轨迹的纯 C++ 检查实现。
 * 本文件不执行 QP、不积分关节速度，也不发布任何硬件指令。
 * 检查顺序：现场限值 → 时间/代次 → 数组与名称 → 剩余时域 → 逐轴位置/步长 → 三次包络。
 * 三次 Hermite 曲线转成 Bernstein 控制点后，可用凸包性质保守检查位置、速度和加速度。
 * 这里使用的是异步收到的参考状态；筛查通过不等于未来真实插值绝对满足限制。
 * 调用方仍需验证输入新鲜度、硬件状态和独立停止机制。拒绝原因由 reason 返回。
 */

#include "elfin5_qp_controller/real_robot_command_gate.h"
#include <algorithm>
#include <cmath>
#include <set>

namespace elfin5_qp_controller {
namespace {
// 同时检查长度和有限性，避免数组越界以及 NaN/Inf 绕过普通大小比较。
bool finiteVector(const std::vector<double>& x, std::size_t n) {
  return x.size() == n && std::all_of(x.begin(), x.end(),
                                    [](double v) { return std::isfinite(v); });
}
// 统一填入原因码，使所有拒绝分支都保持“返回 false + 可追查原因”的约定。
bool reject(const char* text, std::string& reason) { reason = text; return false; }
}
// 限制数组必须全部对应六轴；没有完整现场配置就拒绝，不偷偷使用厂家示例值。
bool HardwareLimits::valid(std::string& reason) const {
  const auto n = names.size();
  if (n != 6 || std::set<std::string>(names.begin(), names.end()).size() != n ||
      std::any_of(names.begin(), names.end(), [](const std::string& s){return s.empty();}))
    return reject("six_unique_joint_names_required", reason);
  if (!finiteVector(lower,n) || !finiteVector(upper,n) || !finiteVector(velocity,n) ||
      !finiteVector(acceleration,n) || !finiteVector(step,n))
    return reject("six_finite_site_limits_required", reason);
  // 时域下限必须为正，且不能大于上限；否则后面的导数计算会失去意义。
  if (!std::isfinite(min_horizon) || !std::isfinite(max_horizon) ||
      min_horizon <= 0 || max_horizon < min_horizon)
    return reject("invalid_horizon_limits", reason);
  // 逐轴检查上下界关系；速度、加速度和允许步长都必须严格为正。
  for (std::size_t i=0;i<n;++i)
    if (lower[i]>=upper[i] || velocity[i]<=0 || acceleration[i]<=0 || step[i]<=0)
      return reject("invalid_joint_limits", reason);
  return true;
}
// 显式按名称查找，兼容上游关节数组乱序；允许真实 JointState 包含额外关节。
bool orderedPositions(const std::vector<std::string>& expected,
                      const std::vector<std::string>& names,
                      const std::vector<double>& positions,
                      std::vector<double>& result) {
  if (positions.size()!=names.size() ||
      std::set<std::string>(names.begin(),names.end()).size()!=names.size()) return false;
  result.clear();
  for (const auto& name:expected) {
    const auto it=std::find(names.begin(),names.end(),name);
    if (it==names.end()) return false;
    const double value=positions[std::distance(names.begin(),it)];  // 取对应名称的值，而非原数组的同序号值。
    if (!std::isfinite(value)) return false;
    result.push_back(value);
  }
  return true;
}
// 消息年龄发现传输/源数据过期，接收年龄发现发布器中断；两项必须同时满足。
// 墙钟年龄不是单调时钟保证，墙钟回退产生负年龄时也会拒绝。
bool fresh(double now, double stamp, double wall_age, double timeout, double future) {
  return std::isfinite(now) && std::isfinite(stamp) && std::isfinite(wall_age) &&
         std::isfinite(timeout) && timeout>0 && stamp>0 && wall_age>=0 &&
         wall_age<=timeout && now-stamp>=-future && now-stamp<=timeout;
}
// 本函数只检查一个已经由 ROS 网关确认形状的候选端点，不做状态机转换。
bool checkCommand(const HardwareLimits& l, const HardwareCommand& c,
                  const std::vector<double>& actual, const std::vector<double>& ref,
                  const std::vector<double>& ref_v, double now, double epoch,
                  double previous_stamp, double max_age, std::string& reason) {
  if (!l.valid(reason)) return false;
  const auto n=l.names.size();
  // 命令须晚于本次激活 epoch 和上次已放行时间；候选路径不容许未来时间戳。
  if (!std::isfinite(c.stamp) || c.stamp<=epoch || c.stamp<=previous_stamp ||
      !fresh(now,c.stamp,0,max_age,0.0))
    return reject("old_future_or_replayed_candidate",reason);
  if (!std::isfinite(c.horizon) || c.horizon<l.min_horizon || c.horizon>l.max_horizon)
    return reject("invalid_command_horizon",reason);
  if (c.names.size()!=n || !finiteVector(actual,n) || !finiteVector(ref,n) ||
      !finiteVector(ref_v,n)) return reject("invalid_reference",reason);
  std::vector<double> q,v;
  if (!orderedPositions(l.names,c.names,c.positions,q) ||
      !orderedPositions(l.names,c.names,c.velocities,v))
    return reject("invalid_candidate_arrays",reason);
  // 扣掉命令传输年龄得到剩余执行时间；不能把原时域全部当作可用时间。
  const double duration=c.horizon-(now-c.stamp);
  if (duration<l.min_horizon) return reject("insufficient_remaining_horizon",reason);
  for (std::size_t i=0;i<n;++i) {
    if (actual[i]<l.lower[i] || actual[i]>l.upper[i] ||
        ref[i]<l.lower[i] || ref[i]>l.upper[i] ||
        q[i]<l.lower[i] || q[i]>l.upper[i])
      return reject("position_limit",reason);
    // 同时限制端点离真实位置及参考位置的偏移，避免其中一个状态落后时产生大跳变。
    if (std::abs(q[i]-actual[i])>l.step[i] || std::abs(q[i]-ref[i])>l.step[i])
      return reject("endpoint_step_limit",reason);
    // Bernstein hull of a cubic Hermite segment, using the observed controller
    // reference as start. This is a conservative screen, not proof of the
    // controller's future interpolation (state and execution are asynchronous).
    // 对时域 T，四个控制点为 q0、q0+T*v0/3、q1−T*v1/3、q1。
    // 曲线位于这些点的凸包中；控制点全在位置限位内是充分条件，但不是必要条件。
    const double b[4]={ref[i],ref[i]+duration*ref_v[i]/3,
                       q[i]-duration*v[i]/3,q[i]};
    for (double p:b)
      if (p<l.lower[i] || p>l.upper[i]) return reject("cubic_position_envelope",reason);
    double dv[3];
    for (int j=0;j<3;++j) {
      // 一阶导数的三个 Bernstein 控制点，除以 T 后单位为 rad/s。
      dv[j]=3*(b[j+1]-b[j])/duration;
      if (std::abs(dv[j])>l.velocity[i]) return reject("cubic_velocity_envelope",reason);
    }
    // 二阶导数的两个控制点为 2*(dv[j+1]−dv[j])/T，单位 rad/s²。
    // 这类包络检查可能保守拒绝；不要通过静默改写端点来绕过。
    for (int j=0;j<2;++j)
      if (std::abs(2*(dv[j+1]-dv[j])/duration)>l.acceleration[i])
        return reject("cubic_acceleration_envelope",reason);
  }
  reason="accepted"; return true;  // 只有全部检查通过后才报告接受；实际发布仍由网关决定。
}
}  // namespace elfin5_qp_controller
