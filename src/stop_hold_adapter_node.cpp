// 站点受控停止请求/反馈确认：不自动使能，不替代急停或执行层看门狗。
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <std_srvs/Trigger.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <mutex>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

class StopHoldAdapter {
  using Clock = std::chrono::steady_clock;
  struct Feedback {
    bool valid=false, still=false;
    ros::Time stamp;
    Clock::time_point received;
  } joints_, desired_;
  ros::NodeHandle n_, p_{"~"};
  ros::Publisher command_;
  ros::Subscriber joints_in_, state_in_;
  ros::ServiceServer service_;
  std::vector<std::string> names_;
  double threshold_=0.005, duration_=0.2, timeout_=1.0, feedback_timeout_=0.3;
  std::mutex data_mutex_, request_mutex_;
  std::condition_variable changed_;
  ros::Time request_stamp_;
  bool quiet_=false;
  Clock::time_point quiet_since_;

  bool fresh(const Feedback& f, const Clock::time_point now, const ros::Time& ros_now) const {
    const double age=std::chrono::duration<double>(now-f.received).count();
    const double source_age=(ros_now-f.stamp).toSec();
    return f.valid && f.stamp>request_stamp_ && age>=0 && age<=feedback_timeout_ &&
           source_age>=-0.01 && source_age<=feedback_timeout_;
  }
  void updateQuiet(const Clock::time_point now) {
    const ros::Time ros_now=ros::Time::now();
    const bool valid=fresh(joints_,now,ros_now) && fresh(desired_,now,ros_now) &&
                     joints_.still && desired_.still;
    if (!valid) quiet_=false;
    else if (!quiet_) { quiet_=true; quiet_since_=now; }
  }
  void receive(const std::vector<std::string>& names, const std::vector<double>& velocities,
               const ros::Time& stamp, Feedback& feedback) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    bool valid=names.size()==velocities.size() && !stamp.isZero() &&
               (feedback.stamp.isZero() || stamp>feedback.stamp);
    std::set<std::string> unique(names.begin(),names.end());
    valid=valid && unique.size()==names.size();
    double peak=0;
    for (const auto& name:names_) {
      const auto found=std::find(names.begin(),names.end(),name);
      if (found==names.end()) { valid=false; break; }
      const std::size_t index=static_cast<std::size_t>(found-names.begin());
      if (index>=velocities.size() || !std::isfinite(velocities[index])) { valid=false; break; }
      peak=std::max(peak,std::abs(velocities[index]));
    }
    feedback.valid=valid; feedback.still=valid && peak<threshold_;
    if (valid) feedback.stamp=stamp;
    feedback.received=Clock::now();
    updateQuiet(feedback.received); changed_.notify_all();
  }
  void onJoint(const sensor_msgs::JointState::ConstPtr& message) {
    receive(message->name,message->velocity,message->header.stamp,joints_);
  }
  void onState(const control_msgs::JointTrajectoryControllerState::ConstPtr& message) {
    receive(message->joint_names,message->desired.velocities,message->header.stamp,desired_);
  }
  bool stop(std_srvs::Trigger::Request&,std_srvs::Trigger::Response& response) {
    response.success=false;
    std::unique_lock<std::mutex> request(request_mutex_,std::try_to_lock);
    if (!request.owns_lock()) { response.message="已有停止请求处理中"; return true; }
    if (!command_.getNumSubscribers()) { response.message="停止命令没有订阅者"; return true; }
    std::unique_lock<std::mutex> lock(data_mutex_);
    request_stamp_=ros::Time::now(); quiet_=false;
    if (request_stamp_.isZero()) { response.message="ROS时间尚未有效"; return true; }
    ros::Time last_ros=request_stamp_;
    const auto deadline=Clock::now()+std::chrono::duration<double>(timeout_);
    try { command_.publish(trajectory_msgs::JointTrajectory()); }
    catch (const std::exception& e) { response.message=std::string("停止请求发布失败：")+e.what(); return true; }
    while (ros::ok()) {
      const auto now=Clock::now();
      if (now>=deadline) break;
      const ros::Time ros_now=ros::Time::now();
      if (ros_now<last_ros) { response.message="停止确认期间ROS时间回退"; return true; }
      last_ros=ros_now;
      if (!command_.getNumSubscribers()) { response.message="停止确认期间命令通路断开"; return true; }
      updateQuiet(now);
      // 需要两路都实际观察到跨越确认时长的新样本，不能等一帧零速变“持续”。
      if (quiet_ && std::chrono::duration<double>(joints_.received-quiet_since_).count()>=duration_ &&
          std::chrono::duration<double>(desired_.received-quiet_since_).count()>=duration_) {
        response.success=true;
        response.message="请求后的六轴反馈与JTC期望速度已持续低于阈值；不是安全急停认证";
        return true;
      }
      changed_.wait_for(lock,std::chrono::milliseconds(5));
    }
    response.message="未能在单调时间期限内用新鲜反馈确认持续低速";
    return true;
  }
public:
  StopHoldAdapter() {
    p_.getParam("joint_names",names_);
    if (names_.empty()) for (int i=1;i<=6;++i) names_.push_back("elfin_joint"+std::to_string(i));
    if (names_.size()!=6 || std::set<std::string>(names_.begin(),names_.end()).size()!=6 ||
        std::find(names_.begin(),names_.end(),std::string())!=names_.end())
      throw std::runtime_error("停止适配器需要六个不同的关节名");
    p_.param("still_velocity",threshold_,0.005); p_.param("confirm_duration",duration_,0.2);
    p_.param("confirm_timeout",timeout_,1.0); p_.param("feedback_timeout",feedback_timeout_,0.3);
    for (double value:{threshold_,duration_,timeout_,feedback_timeout_})
      if (!std::isfinite(value) || value<=0) throw std::runtime_error("停止确认参数必须有限且为正数");
    if (duration_>=timeout_) throw std::runtime_error("持续低速确认时间必须小于总超时");
    std::string command_topic,joints_topic,state_topic,service_name;
    p_.param<std::string>("command_topic",command_topic,"/elfin_arm_controller/command");
    p_.param<std::string>("joint_states_topic",joints_topic,"/joint_states");
    p_.param<std::string>("controller_state_topic",state_topic,"/elfin_arm_controller/state");
    p_.param<std::string>("stop_service",service_name,"/elfin5_site/stop_hold");
    command_=n_.advertise<trajectory_msgs::JointTrajectory>(command_topic,1,false);
    joints_in_=n_.subscribe(joints_topic,1,&StopHoldAdapter::onJoint,this);
    state_in_=n_.subscribe(state_topic,1,&StopHoldAdapter::onState,this);
    service_=n_.advertiseService(service_name,&StopHoldAdapter::stop,this);
    ROS_WARN("停止适配器未经过现场停车验证；启动不发布运动命令、不使能驱动");
  }
};

int main(int argc,char** argv) {
  ros::init(argc,argv,"elfin5_stop_hold_adapter");
  StopHoldAdapter adapter;
  ros::AsyncSpinner spinner(2);  // 服务等待时仍须处理反馈回调。
  spinner.start(); ros::waitForShutdown();
  return 0;
}
