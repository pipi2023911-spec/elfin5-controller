/*
 * test_real_robot_command_gate.cpp
 * 纯命令检查核心的 GoogleTest 测试。
 * 使用人工构造的六轴限值和轨迹，不连接ROS master、EtherCAT或真实机械臂。
 * 关注拒绝行为：旧/未来时间、NaN、重复名称、越界、端点跳变与插值包络超限。
 * 这里证明的是纯检查函数的行为，不能代替网关状态机、控制器插值或物理停止验证。
 * 数值仅是测试夹具，不得复制为现场硬件限值。
 */

#include "elfin5_qp_controller/real_robot_command_gate.h"
#include <gtest/gtest.h>
#include <limits>
namespace eq=elfin5_qp_controller;
// 构造便于计算的六轴假模型：±2rad位置、1rad/s速度和10rad/s²加速度。
eq::HardwareLimits limits() {
  eq::HardwareLimits l;
  for(int i=1;i<=6;++i)l.names.push_back("j"+std::to_string(i));
  l.lower.assign(6,-2);l.upper.assign(6,2);l.velocity.assign(6,1);
  l.acceleration.assign(6,10);l.step.assign(6,0.1);return l;
}
// 生成较小且可接受的候选，其他测试每次只改变一个条件以定位拒绝原因。
eq::HardwareCommand command() {
  eq::HardwareCommand c;c.names=limits().names;c.stamp=10;c.horizon=.1;
  c.positions.assign(6,.001);c.velocities.assign(6,.01);return c;
}
// 固定当前时刻、激活代次和上一命令时间，让测试不受实际系统时钟影响。
bool check(eq::HardwareCommand c,std::string& reason) {
  return eq::checkCommand(limits(),c,std::vector<double>(6,0),
      std::vector<double>(6,0),std::vector<double>(6,0),10.001,9,9.9,.02,reason);
}
// 正常小步应通过；交换名称后也不依赖硬编码数组次序（本夹具各轴值相同）。
TEST(Gate, ValidSmallMotionAndReorderedNames) {
  auto c=command();std::string reason;EXPECT_TRUE(check(c,reason))<<reason;
  std::swap(c.names[0],c.names[1]);EXPECT_TRUE(check(c,reason))<<reason;
}
// 默认空配置必须拒绝，防止未核验限值时误进入运动条件。
TEST(Gate, EmptyLimitsBlockHardware) {
  eq::HardwareLimits l;std::string why;EXPECT_FALSE(l.valid(why));
}
// 分别验证旧时间、未来时间、非有限位置和重复关节名被阻断。
TEST(Gate, RejectsReplayFutureNanAndDuplicate) {
  std::string why;auto c=command();c.stamp=9.8;EXPECT_FALSE(check(c,why));
  c=command();c.stamp=11;EXPECT_FALSE(check(c,why));
  c=command();c.positions[0]=std::numeric_limits<double>::quiet_NaN();EXPECT_FALSE(check(c,why));
  c=command();c.names[1]=c.names[0];EXPECT_FALSE(check(c,why));
}
// 区分位置越界、端点跳变和三次曲线加速度包络超限；最后核对具体原因码。
TEST(Gate, RejectsPositionStepAndSplineAcceleration) {
  std::string why;auto c=command();c.positions[0]=3;EXPECT_FALSE(check(c,why));
  c=command();c.positions[0]=.2;EXPECT_FALSE(check(c,why));
  c=command();c.positions[0]=.03;c.velocities[0]=0;EXPECT_FALSE(check(c,why));
  EXPECT_EQ("cubic_acceleration_envelope",why);
}
// 验证源时间正常但接收间隔过大时仍拒绝，以及零/未来时间戳不能通过。
TEST(Gate, FreshnessUsesBothClocks) {
  EXPECT_TRUE(eq::fresh(10,9.9,.1,.3));
  EXPECT_FALSE(eq::fresh(10,9.9,.4,.3));
  EXPECT_FALSE(eq::fresh(10,0,0,.3));
  EXPECT_FALSE(eq::fresh(10,11,0,.3));
}
// 用不相等且有正负的数值验证名称重排，避免正常用例的等值数据掩盖映射错误。
TEST(Gate, JointMapPreservesSignAndOrder) {
  std::vector<double> q;
  EXPECT_TRUE(eq::orderedPositions({"a","b"},{"b","a"},{-.2,.3},q));
  ASSERT_EQ(2u,q.size());EXPECT_DOUBLE_EQ(.3,q[0]);EXPECT_DOUBLE_EQ(-.2,q[1]);
  EXPECT_FALSE(eq::orderedPositions({"a","b"},{"a","a"},{0,0},q));
}
