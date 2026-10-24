/* 薄网关：人工放行、隔离候选、必要检查、故障保持。
 * 不代理输入、不扫描ROS图、不解析QP诊断、不使能/复位驱动。
 * 缓存姿态来自限位内实际反馈；新鲜度/驱动不满足时不向旧位置回拉。
 * 站点停止保持服务必须经过现场验证，服务回执不等于已确认停稳。
 */
#include "elfin5_qp_controller/real_robot_command_gate.h"
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <controller_manager_msgs/ListControllers.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <urdf/model.h>
#include <atomic>
#include <cmath>
#include <mutex>
#include <set>
#include <stdexcept>
#include <thread>
namespace eq = elfin5_qp_controller;

class RealBridge {
  struct Age {
    ros::Time stamp; ros::SteadyTime receive; bool valid=false;
    void set(const ros::Time& s) { stamp=s; receive=ros::SteadyTime::now(); valid=true; }
    bool ok(double timeout, bool stamped=true) const {
      if(!valid) return false;
      const double age=(ros::SteadyTime::now()-receive).toSec();
      return stamped ? eq::fresh(ros::Time::now().toSec(),stamp.toSec(),age,timeout)
                     : age>=0 && age<=timeout;
    }
  };
  ros::NodeHandle n_,p_{"~"};
  ros::Publisher command_out_,state_out_;
  ros::Subscriber joints_in_,controller_in_,enabled_in_,fault_in_,candidate_in_;
  ros::ServiceServer enable_,reset_; ros::ServiceClient list_client_,stop_client_;
  ros::WallTimer timer_; std::mutex mutex_; std::thread stop_thread_;
  std::atomic<bool> quitting_{false};
  eq::HardwareLimits limits_;
  Age joints_,controller_,enabled_age_,fault_age_;
  std::vector<double> safe_pose_,reference_,reference_velocity_;
  bool driver_enabled_=false,driver_fault_=true,model_valid_=false,reviewed_=false;
  bool stop_pending_=false,stop_busy_=false,stop_failed_=false;
  std::string mode_="OBSERVE",reason_="输出关闭，等待反馈和人工放行";
  std::string controller_name_,controller_type_;
  double timeout_,max_skew_,max_age_,candidate_timeout_,hold_horizon_,last_stamp_=0;
  std::size_t authorization_revision_=0;
  ros::Time epoch_,last_ros_; ros::SteadyTime last_candidate_;
  std::string topic(const std::string& key,const std::string& fallback) {
    std::string v; p_.param(key,v,fallback); return v;
  }
  bool acceptStamp(const ros::Time& s,const Age& a) const {
    return eq::fresh(ros::Time::now().toSec(),s.toSec(),0,timeout_) && (!a.valid || s>a.stamp);
  }
  bool hardwareReady() const {
    // Bool仅有接收时间，不能据此证明PDO新鲜或现场互锁完备。
    return enabled_age_.ok(timeout_,false) && fault_age_.ok(timeout_,false) &&
           driver_enabled_ && !driver_fault_;
  }
  bool feedbackReady() const {
    return joints_.ok(timeout_) && controller_.ok(timeout_) &&
           std::abs((joints_.stamp-controller_.stamp).toSec())<=max_skew_;
  }
  bool ready(std::string& why) const {
    if(ros::Time::isSimTime()) { why="真机入口禁止使用仿真时间"; return false; }
    if(!reviewed_ || !model_valid_) { why="现场保持/停止审查或模型限值未完成"; return false; }
    if(stop_pending_ || stop_busy_ || stop_failed_) { why="停止请求未完成或失败"; return false; }
    if(!hardwareReady()) { why="驱动未使能、报告故障或状态超时"; return false; }
    if(!feedbackReady()) { why="关节/JTC反馈无效、超时或时间差过大"; return false; }
    return true;
  }
  void publishHold(const eq::HardwareCommand& c) {
    trajectory_msgs::JointTrajectory m; m.header.stamp.fromSec(c.stamp); m.joint_names=c.names;
    trajectory_msgs::JointTrajectoryPoint p; p.positions=c.positions; p.velocities=c.velocities;
    p.time_from_start=ros::Duration(c.horizon); m.points.push_back(p); command_out_.publish(m);
  }
  // 离开ACTIVE时仅请求一次保持，不持续追逐后续反馈，不把候选端点当实际姿态。
  void fault(const std::string& why) {
    if(mode_!="ACTIVE") return;
    mode_="FAULT_LATCHED"; reason_=why; ++authorization_revision_;
    eq::HardwareCommand hold; std::string detail;
    if(hardwareReady() && feedbackReady() &&
       eq::makeHoldCommand(limits_,safe_pose_,reference_,reference_velocity_,
                           ros::Time::now().toSec(),hold_horizon_,hold,detail)) {
      publishHold(hold); reason_+="；已请求保持最近有效反馈姿态，尚未确认停稳";
    } else reason_+="；反馈/驱动/插值条件不允许位置保持，不向旧姿态回拉";
    stop_pending_=true; ROS_ERROR_STREAM(reason_<<"；请求站点停止并保持");
  }
  void onJoint(const sensor_msgs::JointState::ConstPtr& m) {
    std::lock_guard<std::mutex> lock(mutex_); std::vector<double> q;
    if(!acceptStamp(m->header.stamp,joints_) ||
       !eq::orderedPositions(limits_.names,m->name,m->position,q) ||
       !eq::positionsWithinLimits(limits_,q)) {
      joints_.valid=false; fault("关节反馈无效或越界"); return;
    }
    safe_pose_=q; joints_.set(m->header.stamp);
  }
  void onController(const control_msgs::JointTrajectoryControllerState::ConstPtr& m) {
    std::lock_guard<std::mutex> lock(mutex_); std::vector<double> q,v,actual;
    if(!acceptStamp(m->header.stamp,controller_) || m->joint_names.size()!=6 ||
       !eq::orderedPositions(limits_.names,m->joint_names,m->actual.positions,actual) ||
       !eq::orderedPositions(limits_.names,m->joint_names,m->desired.positions,q) ||
       !eq::orderedPositions(limits_.names,m->joint_names,m->desired.velocities,v) ||
       !eq::positionsWithinLimits(limits_,actual) || !eq::positionsWithinLimits(limits_,q)) {
      controller_.valid=false; fault("JTC反馈无效或越界"); return;
    }
    reference_=q; reference_velocity_=v; controller_.set(m->header.stamp);
  }
  void onEnabled(const std_msgs::Bool::ConstPtr& m) {
    std::lock_guard<std::mutex> lock(mutex_);
    driver_enabled_=m->data; enabled_age_.set(ros::Time::now());
    if(!driver_enabled_) fault("驱动已撤销使能");
  }
  void onDriverFault(const std_msgs::Bool::ConstPtr& m) {
    std::lock_guard<std::mutex> lock(mutex_);
    driver_fault_=m->data; fault_age_.set(ros::Time::now());
    if(driver_fault_) fault("厂家驱动报告故障");
  }
  void onCommand(const trajectory_msgs::JointTrajectory::ConstPtr& m) {
    std::lock_guard<std::mutex> lock(mutex_); if(mode_!="ACTIVE") return;
    std::string why; if(!ready(why)) { fault(why); return; }
    if(m->points.size()!=1 || !m->points[0].accelerations.empty() || !m->points[0].effort.empty()) {
      fault("候选必须是单点位置/速度轨迹"); return;
    }
    const auto& p=m->points[0]; eq::HardwareCommand c;
    c.stamp=m->header.stamp.toSec(); c.horizon=p.time_from_start.toSec();
    c.names=m->joint_names; c.positions=p.positions; c.velocities=p.velocities;
    if(!eq::checkCommand(limits_,c,safe_pose_,reference_,reference_velocity_,
                        ros::Time::now().toSec(),epoch_.toSec(),last_stamp_,max_age_,why)) {
      fault("候选命令检查失败："+eq::commandReasonChinese(why)); return;
    }
    command_out_.publish(m); last_stamp_=c.stamp; last_candidate_=ros::SteadyTime::now();
  }
  // 仅在人工放行时查询控制器；不每周期查询ROS图。
  bool controllerReady() {
    controller_manager_msgs::ListControllers call; if(!list_client_.call(call)) return false;
    for(const auto& c:call.response.controller) if(c.name==controller_name_) {
      std::set<std::string> names; bool ok=!c.claimed_resources.empty();
      for(const auto& r:c.claimed_resources) {
        ok=ok && r.hardware_interface=="hardware_interface::PositionJointInterface";
        names.insert(r.resources.begin(),r.resources.end());
      }
      return c.state=="running" && c.type==controller_type_ && ok &&
             names==std::set<std::string>(limits_.names.begin(),limits_.names.end());
    }
    return false;
  }
  bool enable(std_srvs::SetBool::Request& req,std_srvs::SetBool::Response& res) {
    if(!req.data) {
      std::lock_guard<std::mutex> lock(mutex_); ++authorization_revision_; fault("人工关闭输出");
      if(mode_=="FAULT_LATCHED" && stop_failed_ && !stop_busy_ && !stop_pending_) stop_pending_=true;
      res.success=true; res.message="候选输出已关闭；保持/停止结果请查看网关状态"; return true;
    }
    // 网络查询在状态锁外，随后重新检查实时输入。
    std::size_t revision;
    { std::lock_guard<std::mutex> lock(mutex_); revision=authorization_revision_; }
    const bool external=controllerReady() && stop_client_.exists();
    std::lock_guard<std::mutex> lock(mutex_);
    if(revision!=authorization_revision_) { res.message="查询期间发生关闭/复位，请重新人工放行"; return true; }
    if(mode_=="FAULT_LATCHED") { res.message="故障已锁存，请确认停稳后人工reset"; return true; }
    if(mode_=="ACTIVE") { res.success=true; res.message="输出已放行"; return true; }
    if(!external) { res.message="位置控制器未运行、资源不符或站点保持服务不存在"; return true; }
    std::string why; if(!ready(why)) { res.message=why; return true; }
    mode_="ACTIVE"; epoch_=ros::Time::now(); last_stamp_=epoch_.toSec();
    last_candidate_=ros::SteadyTime::now(); reason_="人工放行成功，不执行驱动使能";
    res.success=true; res.message=reason_; return true;
  }
  bool reset(std_srvs::Trigger::Request&,std_srvs::Trigger::Response& res) {
    std::lock_guard<std::mutex> lock(mutex_);
    if(mode_=="ACTIVE" || stop_pending_ || stop_busy_ || stop_failed_) {
      res.message="输出活动或停止尚未成功应答，不能复位"; return true;
    }
    mode_="OBSERVE"; last_stamp_=0; ++authorization_revision_; reason_="已复位，输出仍关闭，须再次人工放行";
    res.success=true; res.message=reason_; return true;
  }
  void tick(const ros::WallTimerEvent&) {
    std::lock_guard<std::mutex> lock(mutex_); const auto now=ros::Time::now();
    if(!last_ros_.isZero() && now<last_ros_) {
      joints_.valid=controller_.valid=enabled_age_.valid=fault_age_.valid=false; fault("ROS时间回退");
    }
    last_ros_=now; std::string why; const bool ok=ready(why);
    if(mode_=="ACTIVE") {
      if(!ok) fault(why);
      else if((ros::SteadyTime::now()-last_candidate_).toSec()>candidate_timeout_) fault("候选轨迹超时");
    }
    std_msgs::String state; const auto display=mode_=="OBSERVE" && ok ? "READY" : mode_;
    state.data=display+": "+(mode_=="OBSERVE" ? (ok?"条件满足，等待人工放行":why) : reason_);
    state_out_.publish(state);
  }
  void stopLoop() {
    while(!quitting_ && ros::ok()) {
      bool run=false;
      { std::lock_guard<std::mutex> lock(mutex_);
        if(stop_pending_) { stop_pending_=false; stop_busy_=true; run=true; } }
      if(run) {
        std_srvs::Trigger call; const bool ok=stop_client_.call(call) && call.response.success;
        std::lock_guard<std::mutex> lock(mutex_); stop_busy_=false; stop_failed_=!ok;
        reason_+=ok?"；站点停止保持已应答，仍需确认实际停稳":"；站点停止保持失败，须现场独立停止";
        if(!ok) ROS_ERROR_STREAM(reason_);
      }
      ros::WallDuration(0.01).sleep();
    }
  }
public:
  RealBridge() {
    p_.param("input_timeout",timeout_,0.3); p_.param("max_input_skew",max_skew_,0.08);
    p_.param("candidate_max_age",max_age_,0.02); p_.param("candidate_timeout",candidate_timeout_,0.15);
    p_.param("hold_horizon",hold_horizon_,0.1); p_.param("site_review_complete",reviewed_,false);
    p_.param<std::string>("controller_name",controller_name_,"elfin_arm_controller");
    p_.param<std::string>("controller_type",controller_type_,"position_controllers/JointTrajectoryController");
    p_.getParam("joint_names",limits_.names); p_.getParam("position_lower",limits_.lower);
    p_.getParam("position_upper",limits_.upper); p_.getParam("velocity_limits",limits_.velocity);
    p_.getParam("acceleration_limits",limits_.acceleration); p_.getParam("max_endpoint_step",limits_.step);
    p_.param("min_horizon",limits_.min_horizon,0.02); p_.param("max_horizon",limits_.max_horizon,0.15);
    for(double v:{timeout_,max_skew_,max_age_,candidate_timeout_,hold_horizon_})
      if(!std::isfinite(v)||v<=0) throw std::runtime_error("时间参数必须为有限正数");
    if(limits_.names.size()!=6 || std::set<std::string>(limits_.names.begin(),limits_.names.end()).size()!=6)
      throw std::runtime_error("必须配置六个不同关节名");
    if(hold_horizon_<limits_.min_horizon || hold_horizon_>limits_.max_horizon)
      throw std::runtime_error("保持时域必须位于轨迹时域上下限内");
    std::string why,xml; urdf::Model model; n_.getParam("/robot_description",xml);
    model_valid_=limits_.valid(why) && model.initString(xml);
    if(model_valid_) for(std::size_t i=0;i<6;++i) {
      const auto j=model.getJoint(limits_.names[i]);
      if(!j || j->type!=urdf::Joint::REVOLUTE || !j->limits ||
         limits_.lower[i]<j->limits->lower || limits_.upper[i]>j->limits->upper ||
         limits_.velocity[i]>j->limits->velocity) { model_valid_=false; break; }
    }
    if(!model_valid_) ROS_WARN("限值或模型缺失/不符，网关不能放行");
    const auto output=n_.resolveName(topic("command_topic","/elfin_arm_controller/command"));
    const auto candidate=n_.resolveName(topic("candidate_topic","/elfin5_real/candidate_trajectory"));
    if(output==candidate) throw std::runtime_error("候选通道不得与真实输出相同");
    command_out_=n_.advertise<trajectory_msgs::JointTrajectory>(output,1,false);
    state_out_=p_.advertise<std_msgs::String>("state",1,true);
    joints_in_=n_.subscribe(topic("joint_states_topic","/joint_states"),1,&RealBridge::onJoint,this);
    controller_in_=n_.subscribe(topic("controller_state_topic","/elfin_arm_controller/state"),1,&RealBridge::onController,this);
    enabled_in_=n_.subscribe(topic("enable_state_topic","/elfin_ros_control/elfin/enable_state"),1,&RealBridge::onEnabled,this);
    fault_in_=n_.subscribe(topic("fault_state_topic","/elfin_ros_control/elfin/fault_state"),1,&RealBridge::onDriverFault,this);
    candidate_in_=n_.subscribe(candidate,1,&RealBridge::onCommand,this);
    list_client_=n_.serviceClient<controller_manager_msgs::ListControllers>(topic("list_controllers_service","/controller_manager/list_controllers"));
    stop_client_=n_.serviceClient<std_srvs::Trigger>(topic("stop_service","/elfin5_site/stop_hold"));
    enable_=p_.advertiseService("enable",&RealBridge::enable,this); reset_=p_.advertiseService("reset",&RealBridge::reset,this);
    timer_=n_.createWallTimer(ros::WallDuration(0.01),&RealBridge::tick,this);
    stop_thread_=std::thread(&RealBridge::stopLoop,this);
  }
  ~RealBridge() {
    quitting_=true; stop_client_.shutdown(); list_client_.shutdown();
    if(stop_thread_.joinable()) stop_thread_.join();
  }
};
int main(int argc,char** argv) {
  ros::init(argc,argv,"real_robot_bridge");
  try { RealBridge bridge; ros::AsyncSpinner spinner(3); spinner.start(); ros::waitForShutdown(); }
  catch(const std::exception& e) { ROS_FATAL_STREAM(e.what()); return 1; }
  return 0;
}
