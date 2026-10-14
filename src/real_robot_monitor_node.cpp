/*
 * real_robot_monitor_node.cpp
 * Elfin5 真机只读监视节点：把真实反馈转换为 RViz 可显示、可录包的观测数据。
 * 输入：真实 JointState、world 中的 base_pose 和 target_point、JTC state，以及网关/QP状态。
 * 末端估计：p_W = T_WB * p_B(q_actual)；E = ||p_target_W − p_W||，单位 m。
 * 关节误差：同一条 JTC state 中的 desired.positions − actual.positions，单位 rad。
 * 输出：当前 TCP、标量误差/有效性诊断、六关节误差、RViz标记和有界历史轨迹。
 * TF 模式：pose 由本节点发布 world→base；external 只核对已有 TF 与输入位姿的一致性。
 * 这里不发送运动命令、不写日志文件；原始消息由独立 rosbag 记录，后续脚本负责导出图表。
 * 时间对齐采用限差内最近基座样本和不晚于关节采样的目标，不宣称严格同时测量。
 * 主循环使用 ros::spin() 顺序执行本节点回调，避免并发修改 MoveIt RobotState 和样本队列。
 * 阅读顺序：onBase/onTarget → onJoint → report；关节误差独立看 onController。
 */

#include "elfin5_qp_controller/real_robot_command_gate.h"
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_msgs/TFMessage.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <deque>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <set>
#include <stdexcept>

namespace eq=elfin5_qp_controller;
class RealMonitor {
  // n_ 查找命名空间内的机器人模型和共享话题；p_ 存放 monitor 参数和输出。
  ros::NodeHandle n_,p_{"~"};
  ros::Subscriber joints_,base_,target_,controller_,bridge_,qp_,tf_,tf_static_;
  ros::Publisher error_,point_,diagnostic_,markers_,path_,joint_errors_;
  ros::WallTimer watchdog_;
  tf2_ros::TransformBroadcaster broadcaster_;
  // TF Buffer 供 external 模式按测量时间查询；监听器维护 TF 缓存，不重新解释关节反馈。
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_{buffer_};
  robot_model_loader::RobotModelLoader loader_{"robot_description"};
  moveit::core::RobotModelPtr model_;
  std::unique_ptr<moveit::core::RobotState> state_;
  std::vector<std::string> names_;
  // 保留有限个基座/目标样本，在关节回调到来时选择时间匹配的观测；不是轨迹预测。
  std::deque<geometry_msgs::PoseStamped> bases_;
  std::deque<geometry_msgs::PointStamped> targets_;
  nav_msgs::Path trail_;  // 有限长度的历史位置轨迹，不是待执行规划轨迹。
  // 最近一次有效 TCP，仅用于放置诊断文字；失效时不能当成新的测量点发布。
  geometry_msgs::Point display_point_;
  std::string world_,base_link_,tip_,bridge_state_="OBSERVE",qp_state_="UNKNOWN",tf_mode_;
  double timeout_,skew_,base_tf_position_tolerance_,base_tf_angle_tolerance_;
  int max_points_;
  ros::WallTime joint_receive_;  // 接收墙钟用于检测关节话题停止更新。
  ros::Time last_joint_,last_base_,last_target_,last_controller_,last_clock_;
  bool conflict_=false,base_valid_=false,target_valid_=false;  // TF冲突锁存；基座/目标有效性随新消息更新。
  // 读取可配置输入名，便于接入现场实际话题。
  std::string topic(const std::string& key,const std::string& fallback) {
    std::string value;p_.param(key,value,fallback);return value;
  }
  // 按源时间检查时效；关节发布中断另由 tick() 的墙钟接收年龄检查。
  bool fresh(const ros::Time& t) const {
    return eq::fresh(ros::Time::now().toSec(),t.toSec(),0,timeout_);
  }
  // 任意坐标分量非有限时拒绝，避免在 FK 或误差范数中传播 NaN。
  static bool finite(const geometry_msgs::Point& v) {
    return std::isfinite(v.x)&&std::isfinite(v.y)&&std::isfinite(v.z);
  }
  // 把 ROS Pose 转为刚体变换 T_WB；调用前已检查四元数范数，内部归一化消除小数值误差。
  static Eigen::Isometry3d transform(const geometry_msgs::Pose& p) {
    Eigen::Isometry3d result=Eigen::Isometry3d::Identity();
    result.linear()=Eigen::Quaterniond(p.orientation.w,p.orientation.x,
                                      p.orientation.y,p.orientation.z).normalized().toRotationMatrix();
    result.translation()=Eigen::Vector3d(p.position.x,p.position.y,p.position.z);
    return result;
  }
  // 诊断 KeyValue 用字符串承载数值；17位精度尽量保留 double 信息，便于离线重算。
  void add(diagnostic_msgs::DiagnosticStatus& status,const std::string& key,double value) {
    diagnostic_msgs::KeyValue item;item.key=key;
    std::ostringstream text;text<<std::setprecision(17)<<value;item.value=text.str();
    status.values.push_back(item);
  }
  // 统一发布带时间和有效标志的误差诊断；失效时 error=NaN，避免被误认为误差为零。
  void report(const ros::Time& stamp,bool valid,const std::string& why,double e=0,
              double base_stamp=0,double target_stamp=0) {
    diagnostic_msgs::DiagnosticArray message;message.header.stamp=stamp;
    diagnostic_msgs::DiagnosticStatus status;
    status.name="elfin5_real_tracking";status.hardware_id="model_estimated_tcp";
    status.level=valid?diagnostic_msgs::DiagnosticStatus::OK:diagnostic_msgs::DiagnosticStatus::WARN;
    status.message=why;add(status,"valid",valid?1:0);
    add(status,"position_error_m",valid?e:std::numeric_limits<double>::quiet_NaN());
    // 记录参与计算的基座/目标源时间，离线可直接检查对齐偏差。
    add(status,"base_stamp",base_stamp);add(status,"target_stamp",target_stamp);
    message.status.push_back(status);diagnostic_.publish(message);
    // 失效时单独刷新红色文字；文字的当前显示时间不代表旧 TCP 已被重新测量。
    if(!valid) {
      visualization_msgs::MarkerArray array;visualization_msgs::Marker marker;
      marker.header.frame_id=world_;marker.header.stamp=ros::Time::now();
      marker.ns="real_tracking";marker.id=3;marker.type=visualization_msgs::Marker::TEXT_VIEW_FACING;
      marker.action=visualization_msgs::Marker::ADD;marker.pose.position=display_point_;
      marker.pose.position.z+=0.1;marker.pose.orientation.w=1;marker.scale.z=0.035;
      marker.color.r=1;marker.color.a=1;marker.lifetime=ros::Duration(std::max(0.5,timeout_));
      marker.text="INVALID / STALE: "+why+"\n"+bridge_state_;
      array.markers.push_back(marker);markers_.publish(array);
    }
  }
  // pose 模式下检测其他节点给 base_link_ 提供父变换；包含 /tf 与 /tf_static。
  // 忽略自己的发布，发现竞争来源后锁存 conflict_，需整理发布者并重启。
  void onTF(const ros::MessageEvent<tf2_msgs::TFMessage const>& event) {
    if(tf_mode_!="pose" || event.getPublisherName()==ros::this_node::getName())return;
    for(const auto& t:event.getMessage()->transforms)
      if(t.child_frame_id==base_link_) {
        conflict_=true;ROS_ERROR_THROTTLE(2,"Duplicate base TF: stop other publisher and restart monitor");
      }
  }
  // 检查坐标系、递增时间、有限位置和四元数，缓存真实的 world→base 位姿。
  void onBase(const geometry_msgs::PoseStamped::ConstPtr& m) {
    const auto& q=m->pose.orientation;const double norm=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
    base_valid_=m->header.frame_id==world_ && fresh(m->header.stamp) &&
                m->header.stamp>last_base_ && finite(m->pose.position) &&
                std::isfinite(norm) && std::abs(norm-1)<0.01;
    // 一旦新输入不合格就清空可用基座队列，不悄悄沿用上一次好数据。
    if(!base_valid_) {bases_.clear();return;}
    last_base_=m->header.stamp;bases_.push_back(*m);if(bases_.size()>500)bases_.pop_front();
    // 只在唯一发布者模式下广播，并保留定位源的原时间戳，不能改写为 now。
    if(tf_mode_=="pose" && !conflict_) {
      geometry_msgs::TransformStamped t;t.header=m->header;t.child_frame_id=base_link_;
      t.transform.translation.x=m->pose.position.x;t.transform.translation.y=m->pose.position.y;
      t.transform.translation.z=m->pose.position.z;t.transform.rotation=m->pose.orientation;
      broadcaster_.sendTransform(t);
    }
  }
  // 目标同样必须来自 world；不在这里转换成 base，以免重复变换。
  void onTarget(const geometry_msgs::PointStamped::ConstPtr& m) {
    target_valid_=m->header.frame_id==world_ && finite(m->point) &&
                  fresh(m->header.stamp) && m->header.stamp>last_target_;
    if(!target_valid_){targets_.clear();return;}
    last_target_=m->header.stamp;targets_.push_back(*m);
    if(targets_.size()>500)targets_.pop_front();
  }
  // 缓存网关文字状态用于 RViz，显示本身不改变任何控制状态。
  void onBridge(const std_msgs::String::ConstPtr& m){bridge_state_=m->data;}
  // 只提取 QP 状态名前缀用于显示；它不是与某条轨迹精确配对的求解证明。
  void onQP(const diagnostic_msgs::DiagnosticArray::ConstPtr& m) {
    for(const auto& status:m->status) if(status.name=="elfin5_qp_controller")
      qp_state_=status.message.substr(0,status.message.find(':'));
  }
  // 关节误差独立于末端 FK：从同一条 state 取参考与实际位置，再按名称对应。
  // 保留有符号直接差；不自动做角度取模，以免隐藏有限位关节的分支/零位问题。
  void onController(const control_msgs::JointTrajectoryControllerState::ConstPtr& m) {
    // Same stamp, name map and signed direct difference. Never use future QP endpoint.
    // 这里复用 JointState 容器装误差，position 的物理含义是角度差，绝非机器人实际角度。
    sensor_msgs::JointState out;out.header=m->header;out.name=names_;
    std::vector<double> desired,actual;
    const bool ok=fresh(m->header.stamp) && m->header.stamp>last_controller_ &&
       m->joint_names.size()==names_.size() &&
       eq::orderedPositions(names_,m->joint_names,m->desired.positions,desired) &&
       eq::orderedPositions(names_,m->joint_names,m->actual.positions,actual);
    if(ok) {
      last_controller_=m->header.stamp;
      for(std::size_t i=0;i<names_.size();++i)out.position.push_back(desired[i]-actual[i]);
    // 不合格样本输出六个 NaN，让下游可显式识别无效值，而不是画出虚假的零误差。
    } else out.position.assign(names_.size(),std::numeric_limits<double>::quiet_NaN());
    // A labelled error topic, never remapped into robot_state_publisher.
    joint_errors_.publish(out);
  }
  // 末端估计以真实关节采样触发；任一输入缺失、过期或 TF 冲突时仅报告无效。
  void onJoint(const sensor_msgs::JointState::ConstPtr& m) {
    joint_receive_=ros::WallTime::now();
    std::vector<double> q;
    if(!fresh(m->header.stamp) || m->header.stamp<=last_joint_ ||
       !eq::orderedPositions(names_,m->name,m->position,q)) {
      report(ros::Time::now(),false,"invalid_joint_feedback");return;
    }
    last_joint_=m->header.stamp;
    if(!base_valid_ || !target_valid_ || bases_.empty() || targets_.empty() || conflict_) {
      report(m->header.stamp,false,"missing_input_or_tf_conflict");return;
    }
    // 遍历有界缓存选择时间差最小的基座样本；它可能稍早或稍晚，但必须在 skew_ 范围内。
    const geometry_msgs::PoseStamped* b=nullptr;
    double nearest=skew_+1;
    for(const auto& item:bases_) {
      const double dt=std::abs((item.header.stamp-m->header.stamp).toSec());
      if(dt<nearest){nearest=dt;b=&item;}
    }
    // 目标按因果顺序选取：不能用关节采样时刻之后才发布的新目标评估过去误差。
    const geometry_msgs::PointStamped* target=nullptr;
    for(const auto& item:targets_)
      if(item.header.stamp<=m->header.stamp)target=&item;
    // 最近样本仍可能太远或已过期；未满足条件时不插值、不补零、不发布新的TCP位置。
    if(!b || nearest>skew_ || !fresh(b->header.stamp) ||
       !target || !fresh(target->header.stamp)) {
      report(m->header.stamp,false,"unaligned_or_stale_base_target");return;
    }
    // wb 是基座到 world 的刚体变换；后面的乘法方向与输入定义保持一致。
    const auto wb=transform(b->pose);
    // external 模式不广播基座 TF，而是要求外部 TF 在同一定位时间能查到且与 base_pose 一致。
    if(tf_mode_=="external") {
      try {
        // 零等待查询避免关节回调阻塞；此时查不到就报无效，等下一帧再处理。
        const auto t=buffer_.lookupTransform(world_,base_link_,b->header.stamp,ros::Duration(0));
        // 拒绝只由静态变换组成的基座结果，避免把移动基座错误固定在 world。
        if(t.header.stamp.isZero() || std::abs((t.header.stamp-b->header.stamp).toSec())>skew_) {
          report(m->header.stamp,false,"external_base_tf_not_dynamic_or_aligned");return;
        }
        geometry_msgs::Pose p;p.position.x=t.transform.translation.x;
        p.position.y=t.transform.translation.y;p.position.z=t.transform.translation.z;
        p.orientation=t.transform.rotation;
        const auto check=transform(p);
        if((check.translation()-wb.translation()).norm()>base_tf_position_tolerance_ ||
           Eigen::AngleAxisd(check.linear().transpose()*wb.linear()).angle()>base_tf_angle_tolerance_)
          {report(m->header.stamp,false,"external_tf_base_pose_mismatch");return;}
      } catch(const tf2::TransformException&) {
        report(m->header.stamp,false,"external_tf_unavailable");return;
      }
    }
    // 向 MoveIt 写入真实关节位置并更新 FK；没有使用候选或期望关节位置。
    state_->setVariablePositions(names_,q);state_->update();
    // 先用 T_model_base 的逆左乘 T_model_tip，得到相对 base_link_ 的 TCP。
    // 因此厂家 base_link→elfin_base 的固定旋转只由模型计算一次，不再手工叠加90°。
    const Eigen::Vector3d local=(state_->getGlobalLinkTransform(base_link_).inverse() *
                                state_->getGlobalLinkTransform(tip_)).translation();
    // 将基座坐标 TCP 转到固定 world；包含基座旋转和平移的共同影响。
    const Eigen::Vector3d position=wb*local;
    const Eigen::Vector3d goal(target->point.x,target->point.y,target->point.z);
    // 只输出位置误差的二范数，不生成 XYZ 三分量误差曲线。
    const double e=(goal-position).norm();
    geometry_msgs::PointStamped current;current.header=m->header;current.header.frame_id=world_;
    current.point.x=position.x();current.point.y=position.y();current.point.z=position.z();
    display_point_=current.point;
    // 当前位置沿用关节采样时间；误差诊断另保存所选基座/目标时间，说明它是估计结果。
    point_.publish(current);std_msgs::Float64 scalar;scalar.data=e;error_.publish(scalar);
    report(m->header.stamp,true,"estimated_from_joint_and_base",e,
           b->header.stamp.toSec(),target->header.stamp.toSec());
    // Path 仅用来显示位置历史，orientation.w=1 是显示占位，不代表控制/测量了末端姿态。
    geometry_msgs::PoseStamped pose;pose.header=current.header;pose.pose.position=current.point;
    pose.pose.orientation.w=1;trail_.header=current.header;trail_.poses.push_back(pose);
    // 限制历史点数，长时间现场运行不会让轨迹缓存无限增长。
    if(trail_.poses.size()>static_cast<std::size_t>(max_points_))trail_.poses.erase(trail_.poses.begin());
    path_.publish(trail_);
    visualization_msgs::MarkerArray array;
    // 四类可视对象使用固定ID更新：0目标点、1估计TCP、2误差连线、3状态文字。
    for(int i=0;i<4;++i) {
      visualization_msgs::Marker marker;marker.header=current.header;marker.ns="real_tracking";
      marker.id=i;marker.action=visualization_msgs::Marker::ADD;marker.pose.orientation.w=1;
      // 设置有限生命周期，输入停止后旧的活动标记不会永久冒充有效显示。
      marker.color.a=1;marker.lifetime=ros::Duration(timeout_);
      if(i<2) {
        marker.type=visualization_msgs::Marker::SPHERE;
        marker.scale.x=marker.scale.y=marker.scale.z=0.025;
        marker.pose.position=i==0?target->point:current.point;
        marker.color.r=i==0?1:0;marker.color.g=i==1?1:0;
      } else if(i==2) {
        marker.type=visualization_msgs::Marker::LINE_LIST;marker.scale.x=0.003;
        marker.color.r=marker.color.g=1;marker.points={target->point,current.point};
      } else {
        marker.type=visualization_msgs::Marker::TEXT_VIEW_FACING;marker.scale.z=0.035;
        marker.color.r=marker.color.g=marker.color.b=1;marker.pose.position=current.point;
        marker.pose.position.z+=0.10;
        std::ostringstream text;text<<bridge_state_<<"\nQP: "<<qp_state_
          <<"\nE = "<<std::fixed<<std::setprecision(2)<<e*1000<<" mm";
        marker.text=text.str();
      }
      array.markers.push_back(marker);
    }
    markers_.publish(array);
  }
  // 墙钟监督与ROS时间回退处理；没有新 JointState 时也能发出超时诊断。
  void tick(const ros::WallTimerEvent&) {
    const auto now=ros::Time::now();
    if(!last_clock_.isZero() && now<last_clock_) {
      // 时间回退后旧缓存不再具有可靠顺序，清空后等待新一轮时间一致的输入。
      bases_.clear();targets_.clear();trail_.poses.clear();
      last_joint_=last_base_=last_target_=last_controller_=ros::Time();
      base_valid_=target_valid_=false;
    }
    last_clock_=now;
    if(joint_receive_.isZero() || (ros::WallTime::now()-joint_receive_).toSec()>timeout_ ||
       !fresh(last_base_) || !fresh(last_target_))
      report(now,false,"feedback_timeout");
  }
public:
  // 读取坐标系/时效配置并加载同一个硬件模型；配置不合格时启动失败，不带病显示。
  RealMonitor() {
    p_.param<std::string>("world_frame",world_,"world");
    p_.param<std::string>("base_link",base_link_,"elfin_base_link");
    p_.param<std::string>("tip_link",tip_,"elfin_end_link");
    p_.param<std::string>("base_tf_mode",tf_mode_,"pose");
    p_.param("input_timeout",timeout_,0.3);p_.param("max_input_skew",skew_,0.08);
    p_.param("path_max_points",max_points_,2000);
    p_.param("base_tf_position_tolerance",base_tf_position_tolerance_,0.001);
    p_.param("base_tf_angle_tolerance",base_tf_angle_tolerance_,0.001);
    p_.getParam("joint_names",names_);
    model_=loader_.getModel();
    if(!model_ || !model_->hasLinkModel(base_link_) || !model_->hasLinkModel(tip_) ||
       names_.size()!=6 || std::set<std::string>(names_.begin(),names_.end()).size()!=6 ||
       (tf_mode_!="pose" && tf_mode_!="external") ||
       !std::isfinite(timeout_) || timeout_<=0 || !std::isfinite(skew_) || skew_<=0 ||
       !std::isfinite(base_tf_position_tolerance_) || base_tf_position_tolerance_<=0 ||
       !std::isfinite(base_tf_angle_tolerance_) || base_tf_angle_tolerance_<=0 ||
       max_points_<1)throw std::runtime_error("Invalid monitor configuration/model");
    // 逐个确认配置名称存在于模型变量中；避免 setVariablePositions 时错误匹配。
    const auto& variables=model_->getVariableNames();
    for(const auto& name:names_)if(std::find(variables.begin(),variables.end(),name)==variables.end())
      throw std::runtime_error("Unknown joint variable: "+name);
    // 默认状态只用于初始化模型；实际发布TCP前必须收到并写入真实六轴位置。
    state_.reset(new moveit::core::RobotState(model_));state_->setToDefaultValues();
    point_=p_.advertise<geometry_msgs::PointStamped>("current_point",1);
    error_=p_.advertise<std_msgs::Float64>("tracking_error",1);
    diagnostic_=p_.advertise<diagnostic_msgs::DiagnosticArray>("tracking_state",20);
    // 此误差话题只供监视和录包，不能重映射给 robot_state_publisher。
    joint_errors_=p_.advertise<sensor_msgs::JointState>("joint_tracking_error",20);
    markers_=p_.advertise<visualization_msgs::MarkerArray>("markers",1);
    path_=p_.advertise<nav_msgs::Path>("path",1);
    joints_=n_.subscribe(topic("joint_states_topic","/joint_states"),10,&RealMonitor::onJoint,this);
    base_=n_.subscribe(topic("base_pose_topic","/base_pose"),10,&RealMonitor::onBase,this);
    target_=n_.subscribe(topic("target_point_topic","/target_point"),10,&RealMonitor::onTarget,this);
    controller_=n_.subscribe(topic("controller_state_topic","/elfin_arm_controller/state"),20,&RealMonitor::onController,this);
    bridge_=n_.subscribe("bridge/state",1,&RealMonitor::onBridge,this);
    qp_=n_.subscribe("qp_candidate/state",1,&RealMonitor::onQP,this);
    // 同时监听动态/静态TF，识别厂家固定world模型仍在运行的冲突情况。
    tf_=n_.subscribe("/tf",50,&RealMonitor::onTF,this);
    tf_static_=n_.subscribe("/tf_static",50,&RealMonitor::onTF,this);
    watchdog_=n_.createWallTimer(ros::WallDuration(0.2),&RealMonitor::tick,this);
  }
};
// 使用顺序 ROS 回调维护 FK 和缓存的一致性；初始化异常使 required 节点退出。
int main(int argc,char** argv) {
  ros::init(argc,argv,"real_robot_monitor");
  try {RealMonitor monitor;ros::spin();}
  catch(const std::exception& e){ROS_FATAL_STREAM(e.what());return 1;}
  return 0;
}
