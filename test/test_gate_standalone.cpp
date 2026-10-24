// 独立C++核心回归：无需ROS/Eigen/GoogleTest，不验证物理停车。
#include "elfin5_qp_controller/real_robot_command_gate.h"
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
namespace eq=elfin5_qp_controller;
static int checks=0;
void require(bool ok,const char* description) {
  ++checks; if(!ok) throw std::runtime_error(description);
}
eq::HardwareLimits limits() {
  eq::HardwareLimits l;
  for(int i=1;i<=6;++i) l.names.push_back("j"+std::to_string(i));
  l.lower.assign(6,-2); l.upper.assign(6,2); l.velocity.assign(6,1);
  l.acceleration.assign(6,10); l.step.assign(6,.1); return l;
}
int main() {
  try {
    auto l=limits(); std::string why; eq::HardwareCommand hold;
    std::vector<double> q{.01,-.02,.03,-.04,.05,-.06},zero(6,0),out;
    require(eq::makeHoldCommand(l,q,q,zero,10,.1,hold,why),"有效反馈保持应通过");
    require(hold.positions==q,"保持必须来自实际反馈，不是未来候选");
    require(hold.velocities==zero,"保持端点速度必须为零");
    require(hold.names==l.names && hold.stamp==10 && hold.horizon==.1,"保持名称与新时间有效");
    auto unsafe=q; unsafe[2]=3;
    require(!eq::makeHoldCommand(l,unsafe,q,zero,10,.1,hold,why),"越界保持须拒绝");
    require(hold.positions.empty(),"失败不能留下可误发的旧保持命令");
    unsafe=q; unsafe[1]=std::numeric_limits<double>::quiet_NaN();
    require(!eq::makeHoldCommand(l,unsafe,q,zero,10,.1,hold,why),"NaN保持须拒绝");
    require(!eq::makeHoldCommand(l,q,q,zero,10,0,hold,why),"零时域须拒绝");
    require(!eq::makeHoldCommand(l,q,q,zero,10,1,hold,why),"过长时域须拒绝");
    require(!eq::makeHoldCommand(l,q,q,zero,0,.1,hold,why),"零时间戳须拒绝");
    require(!eq::makeHoldCommand(l,q,q,zero,std::numeric_limits<double>::quiet_NaN(),.1,hold,why),"NaN时间须拒绝");
    auto moving=zero; moving[0]=1;
    require(!eq::makeHoldCommand(l,q,q,moving,10,.02,hold,why),"不能绕过保持加速度检查");
    require(!eq::makeHoldCommand(l,q,std::vector<double>(6,.5),zero,10,.1,hold,why),"过大回拉须拒绝");
    require(eq::positionsWithinLimits(l,q),"实际位置限位判断");
    require(!eq::positionsWithinLimits(eq::HardwareLimits(),q),"空限值不能缓存安全姿态");
    require(!eq::positionsWithinLimits(l,std::vector<double>(5,0)),"缺轴须拒绝");
    require(eq::orderedPositions({"a","b"},{"b","a"},{-.2,.3},out) && out==std::vector<double>({.3,-.2}),"按名称保持顺序/符号");
    require(!eq::orderedPositions({"a","b"},{"a","a"},{0,0},out),"重复关节须拒绝");
    require(eq::fresh(10,9.9,.1,.3),"新鲜反馈");
    require(!eq::fresh(10,9.9,.4,.3),"接收超时");
    require(!eq::fresh(10,9,.1,.3),"源时间超时");
    require(!eq::fresh(10,11,0,.3),"未来反馈");
    require(eq::makeHoldCommand(l,q,q,zero,10,.1,hold,why),"构造测试候选");
    require(eq::checkCommand(l,hold,q,q,zero,10.001,9,9.9,.02,why),"正常候选");
    require(!eq::checkCommand(l,hold,q,q,zero,10.001,10,10,.02,why),"激活前候选须拒绝");
    require(!eq::checkCommand(l,hold,q,q,zero,10.1,9,9.9,.02,why),"旧命令须拒绝");
    require(!eq::checkCommand(l,hold,q,q,zero,9.99,9,9.9,.02,why),"未来命令须拒绝");
    hold.positions[0]=.5;
    require(!eq::checkCommand(l,hold,q,q,zero,10.001,9,9.9,.02,why),"非法跳变须拒绝");
    require(eq::commandReasonChinese(why)=="端点跳变超过限制","面向用户的拒绝原因中文");
    std::cout<<"通过 "<<checks<<" 项独立C++核心断言；未验证ROS或真机停车\n";
    return 0;
  } catch(const std::exception& e) { std::cerr<<"失败："<<e.what()<<"\n"; return 1; }
}
