/*
 * real_robot_bridge_node.cpp
 * Elfin5 真机命令网关：候选轨迹与实际 EtherCAT 执行链之间的唯一规划出口。
 * 输入：真实 joint_states、world 中的 base_pose/target_point、JTC state、硬件诊断和QP诊断；显示/误差监视不参与运动准入。
 * 输出：校验后的 QP 专用输入、网关状态，以及仅 ACTIVE 状态允许发送的原始候选轨迹。
 * 状态机：OBSERVE（条件不足）→ READY（条件齐全）→ 显式 enable → ACTIVE。
 * ACTIVE 遇异常进入 FAULT_LATCHED；reset 只回到观察，数据恢复不能自动恢复运动。
 * 并发：ROS 三线程回调共用 mutex_；控制器/ROS 图查询和停止服务各有独立工作线程。
 * 时间：消息时间用于对齐和防旧命令；墙钟接收年龄及 WallTimer 用于发现回调/发布中断。
 * 重要边界：关闭放行不是物理停止。stop 请求、独立硬件看门狗和现场监督必须单独验证。
 * 不启动 EtherCAT 主站，不清故障、不使能厂家驱动器、不改 QP 或轨迹端点。
 * 阅读顺序：ready → onCommand → enable/reset/fault → 输入回调 → tick/poll/stopLoop → 构造函数。
 */

#include "elfin5_qp_controller/real_robot_command_gate.h"
#include <ros/ros.h>
#include <ros/master.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <controller_manager_msgs/ListControllers.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <std_msgs/String.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <urdf/model.h>
#include <algorithm>
#include <atomic>
#include <cmath>
#include <mutex>
#include <set>
#include <stdexcept>
#include <thread>

namespace eq = elfin5_qp_controller;
// 节点对象持有所有订阅、发布和状态；外部只通过 ROS 接口与它交互。
class RealBridge {
  // 为每类输入分别保存源时间和本机接收时间；valid=false 会立即阻止 ready()。
  struct Age {
    ros::Time stamp;  // 源消息的测量/生成时间。
    ros::WallTime receive;  // 本进程最近一次接受该消息的墙钟时间。
    bool valid=false;  // 默认无效；未收到数据不能误判为正常。
    // 在真正准备放行时重新计算年龄，不能只相信接收回调当时的检查结果。
    bool ok(double timeout) const {
      return valid && eq::fresh(ros::Time::now().toSec(),stamp.toSec(),
                               (ros::WallTime::now()-receive).toSec(),timeout);
    }
    void set(const ros::Time& s) {stamp=s;receive=ros::WallTime::now();valid=true;}
  };
  // n_ 使用节点所在命名空间连接公共接口；p_ 使用私有命名空间读取配置/提供服务。
  ros::NodeHandle n_,p_{"~"};
  ros::Publisher joint_out_,base_out_,target_out_,command_out_,state_out_;
  ros::Subscriber joint_in_,base_in_,target_in_,command_in_,controller_in_,hardware_in_,qp_in_;
  ros::ServiceServer enable_,reset_;
  ros::ServiceClient list_client_,stop_client_;
  ros::WallTimer timer_;
  // 所有共享控制状态和命令发布都在同一互斥保护下；原子标志用于工作线程握手。
  std::mutex mutex_;
  std::thread poll_thread_,stop_thread_;
  std::atomic<bool> quitting_{false},stop_requested_{false},stop_in_progress_{false};
  eq::HardwareLimits limits_;
  // 六类控制输入各自独立计时；monitor 只负责观测，不充当第二个准入管理器。
  Age joints_,base_,target_,controller_,hardware_,qp_;
  // actual_ 来自真实 JointState；reference_* 来自下层控制器当前插值参考，均按六轴名称重排。
  std::vector<double> actual_,reference_,reference_velocity_;
  std::string world_,controller_name_,controller_type_,hardware_name_,hardware_id_;
  std::string command_topic_,candidate_topic_,candidate_node_,reason_="startup_observe";
  std::string mode_="OBSERVE";  // 启动/重启不会直接执行轨迹。
  double timeout_,max_skew_,max_candidate_age_,candidate_timeout_,last_stamp_=0;
  ros::Time epoch_,last_ros_;  // 分别记录本次激活边界和用于检测时间回退的上一时刻。
  ros::WallTime last_candidate_wall_,poll_wall_;
  bool reviewed_=false,model_valid_=false,poll_ok_=false;
  bool stop_failed_=false;  // 仅在 mutex_ 下访问；失败回执阻止人工复位，成功回执不等于物理静止。
  // 将可替换的话题/服务名集中从私有参数读出，缺失时使用配置约定的默认名。
  std::string topic(const std::string& key,const std::string& fallback) {
    std::string value; p_.param(key,value,fallback); return value;
  }
  // 拒绝任意坐标分量的 NaN/Inf，防止其传入坐标变换或控制器。
  static bool finitePoint(const geometry_msgs::Point& v) {
    return std::isfinite(v.x)&&std::isfinite(v.y)&&std::isfinite(v.z);
  }
  // 接收阶段要求源时间有效且新鲜；已有有效数据时还要求严格递增，拒绝重复帧。
  bool acceptStamp(const ros::Time& stamp, const Age& age) {
    return eq::fresh(ros::Time::now().toSec(),stamp.toSec(),0,timeout_) &&
           (!age.valid || stamp>age.stamp);
  }
  // 调用者已经持有 mutex_。只有 ACTIVE 才转为锁存故障并发起停止请求；
  // 观察阶段的坏数据只更新原因，不会触发站点停止服务。
  void fault(const std::string& reason) {
    if(mode_=="FAULT_LATCHED") return;  // 保留首个故障及停止结果，后续坏数据不能覆盖。
    reason_=reason;
    if(mode_=="ACTIVE") {
      mode_="FAULT_LATCHED";
      // 只提交请求，由 stopLoop 发起可能阻塞的服务调用，避免在命令回调里等待网络。
      stop_requested_=true;
      ROS_ERROR_STREAM("Gate closed: "<<reason<<"; requesting site stop");
    }
  }
  // 底层检查保留稳定的英文原因码；仅在桥接节点输出 why 前转换为中文。
  static std::string commandReasonChinese(const std::string& code) {
    struct Translation { const char* code; const char* text; };
    static const Translation translations[] = {
      {"six_unique_joint_names_required", "必须配置六个互不重复的关节名称"},
      {"six_finite_site_limits_required", "必须为六个关节配置完整且为有限数值的现场限值"},
      {"invalid_horizon_limits", "轨迹时域上下限配置无效"},
      {"invalid_joint_limits", "关节位置、速度、加速度或步长限值配置无效"},
      {"old_future_or_replayed_candidate", "候选轨迹时间戳过旧、位于未来或违反激活后严格递增要求"},
      {"invalid_command_horizon", "候选轨迹时域无效或超出允许范围"},
      {"invalid_reference", "实际关节位置或控制器参考位置、速度的维度或数值无效"},
      {"invalid_candidate_arrays", "候选轨迹的关节名称、位置或速度数组无效"},
      {"insufficient_remaining_horizon", "候选轨迹剩余执行时间小于允许的最小时域"},
      {"position_limit", "实际位置、参考位置或候选终点超出关节位置限值"},
      {"endpoint_step_limit", "候选终点相对实际位置或参考位置的步长超过限制"},
      {"cubic_position_envelope", "三次插值的位置包络超出关节限位"},
      {"cubic_velocity_envelope", "三次插值的速度包络超过关节速度限制"},
      {"cubic_acceleration_envelope", "三次插值的加速度包络超过关节加速度限制"},
      {"accepted", "候选轨迹检查通过"}
    };
    for(const auto& entry:translations) if(code==entry.code) return entry.text;
    return std::string("未识别的命令检查原因：")+code;
  }
  // 放行条件的统一入口：它只判断条件并给出 why，不主动变成 ACTIVE，也不使能机械臂。
  bool ready(std::string& why) {
    // Do not make parameter-server RPCs in the gate/watchdog callback.
    if(ros::Time::isSimTime()) {why="真机控制禁止使用仿真时间";return false;}
    // 现场审查标志和模型内的真实限值缺一不可；默认配置有意停留在观察状态。
    if(!reviewed_ || !model_valid_) {why="现场审查未完成，或模型及关节限值缺失或无效";return false;}
    if(stop_in_progress_ || stop_requested_) {why="停止请求尚未处理完成";return false;}
    // 控制所需的真实反馈、定位、目标、JTC、硬件及QP状态必须有效且未超时。
    if(!joints_.ok(timeout_) || !base_.ok(timeout_) || !target_.ok(timeout_) ||
       !controller_.ok(timeout_) || !hardware_.ok(timeout_) || !qp_.ok(timeout_)) {
      why="输入数据缺失、无效或已超时";return false;
    }
    // 不同来源即使各自未超时，也可能相差太大；另设跨输入时间差限制。
    if(std::abs((joints_.stamp-base_.stamp).toSec())>max_skew_ ||
       std::abs((joints_.stamp-controller_.stamp).toSec())>max_skew_) {
      why="关节反馈与基座位姿或控制器状态的时间差超过限制";return false;
    }
    // 轮询结果也有有效期：服务调用卡住时，旧的“正常”结果不能无限维持放行。
    if(!poll_ok_ || (ros::WallTime::now()-poll_wall_).toSec()>timeout_) {
      why="控制器状态或命令发布者检查未通过、停止服务不可用，或轮询结果已超时";return false;
    }
    return true;
  }
  // 真实关节输入：检查名称映射和时间后缓存标准顺序位置；原消息转发给候选 QP。
  void onJoint(const sensor_msgs::JointState::ConstPtr& m) {
    // 互斥范围覆盖验证、缓存和转发，防止命令回调读到一半更新的状态。
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<double> q;
    if(!acceptStamp(m->header.stamp,joints_) ||
       !eq::orderedPositions(limits_.names,m->name,m->position,q)) {
      joints_.valid=false;fault("invalid_joint_feedback");return;
    }
    actual_=q;joints_.set(m->header.stamp);joint_out_.publish(m);
  }
  // 基座位姿必须为 world→base。这里只检查输入，不把世界位姿再次求逆或改变坐标系。
  void onBase(const geometry_msgs::PoseStamped::ConstPtr& m) {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto& q=m->pose.orientation;
    // norm 是四元数范数的平方；允许小数值误差，但拒绝零四元数或明显未归一化输入。
    const double norm=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
    if(!acceptStamp(m->header.stamp,base_) || m->header.frame_id!=world_ ||
       !finitePoint(m->pose.position) || !std::isfinite(norm) || std::abs(norm-1)>0.01) {
      base_.valid=false;fault("invalid_base_pose");return;
    }
    base_.set(m->header.stamp);base_out_.publish(m);
  }
  // 目标点同样在固定 world 中；本版固定目标也需上游持续发送带新时间戳的消息。
  void onTarget(const geometry_msgs::PointStamped::ConstPtr& m) {
    std::lock_guard<std::mutex> lock(mutex_);
    if(!acceptStamp(m->header.stamp,target_) || m->header.frame_id!=world_ ||
       !finitePoint(m->point)) {
      target_.valid=false;fault("invalid_target");return;
    }
    target_.set(m->header.stamp);target_out_.publish(m);
  }
  // 从同一 JTC state 提取参考位置/参考速度，作为候选包络的观测起点。
  // 同时验证 actual 数组形状；实际位置边界检查仍采用独立真实 JointState 的 actual_。
  void onController(const control_msgs::JointTrajectoryControllerState::ConstPtr& m) {
    // 互斥范围覆盖验证、缓存和转发，防止命令回调读到一半更新的状态。
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<double> q;
    if(!acceptStamp(m->header.stamp,controller_) || m->joint_names.size()!=6 ||
       !eq::orderedPositions(limits_.names,m->joint_names,m->actual.positions,q) ||
       !eq::orderedPositions(limits_.names,m->joint_names,m->desired.positions,reference_) ||
       !eq::orderedPositions(limits_.names,m->joint_names,m->desired.velocities,reference_velocity_)) {
      controller_.valid=false;fault("invalid_controller_state");return;
    }
    controller_.set(m->header.stamp);
  }
  // 硬件诊断是现场监督适配器的契约，不是猜测的厂家接口。
  // 名称唯一、硬件 ID 匹配且 level=OK 才有效；缺失、未知或告警都阻止放行。
  void onHardware(const diagnostic_msgs::DiagnosticArray::ConstPtr& m) {
    std::lock_guard<std::mutex> lock(mutex_);
    bool ok=acceptStamp(m->header.stamp,hardware_);
    int count=0;
    for(const auto& s:m->status) if(s.name==hardware_name_) {
      ++count; ok=ok && s.hardware_id==hardware_id_ &&
                        s.level==diagnostic_msgs::DiagnosticStatus::OK;
    }
    if(ok && count==1 && !hardware_id_.empty()) hardware_.set(m->header.stamp);
    else {hardware_.valid=false;fault("hardware_interlock_not_ok");}
  }
  // 只允许正常运行、到点保持和奇异区恢复；QP_DEGRADED 不列入白名单。
  // 诊断和候选轨迹并非同一原子消息，因此该检查不能证明逐周期配对。
  void onQP(const diagnostic_msgs::DiagnosticArray::ConstPtr& m) {
    std::lock_guard<std::mutex> lock(mutex_);
    bool ok=acceptStamp(m->header.stamp,qp_);
    int count=0;
    for(const auto& s:m->status) if(s.name=="elfin5_qp_controller") {
      ++count;
      const auto state=s.message.substr(0,s.message.find(':'));
      ok=ok && (state=="RUNNING" || state=="REACHED" || state=="SINGULARITY_RECOVERY");
    }
    if(ok && count==1) qp_.set(m->header.stamp);
    else {qp_.valid=false;fault("qp_not_running_or_degraded");}
  }
  // 候选命令回调是实际轨迹发布的唯一代码路径。
  // 先查状态和全部门禁，再调用纯检查函数；失败即关闭本轮以及后续放行。
  void onCommand(const trajectory_msgs::JointTrajectory::ConstPtr& m) {
    std::lock_guard<std::mutex> lock(mutex_);
    // 观察、就绪、锁存故障时直接丢弃候选，包括原 V2 可能发布的 hold。
    if(mode_!="ACTIVE") return;
    std::string why;
    if(!ready(why)) {fault(why);return;}
    // 限定为 V2 的单点位置+速度协议；不接收额外加速度/力矩字段，以免扩大解释范围。
    if(m->points.size()!=1 || !m->points.front().accelerations.empty() ||
       !m->points.front().effort.empty()) {fault("unexpected_trajectory_shape");return;}
    const auto& point=m->points.front();
    // 将 ROS 消息提取成不依赖 ROS 的检查输入，便于核心逻辑单独测试。
    eq::HardwareCommand c;
    c.stamp=m->header.stamp.toSec();c.horizon=point.time_from_start.toSec();
    c.names=m->joint_names;c.positions=point.positions;c.velocities=point.velocities;
    if(!eq::checkCommand(limits_,c,actual_,reference_,reference_velocity_,
                        ros::Time::now().toSec(),epoch_.toSec(),last_stamp_,
                        max_candidate_age_,why)) {why=commandReasonChinese(why);fault(why);return;}
    // No integration, clipping, retimestamping, or trajectory substitution.
    // 原样发布：不重新积分、不裁剪端点、不重置时间。接受后才推进防重放时间。
    command_out_.publish(m);last_stamp_=c.stamp;
    last_candidate_wall_=ros::WallTime::now();
  }
  // SetBool 服务只管理网关放行，不调用厂家驱动使能。
  // 返回 true 表示 ROS 服务正常处理；业务是否成功由 res.success 表示。
  bool enable(std_srvs::SetBool::Request& req,std_srvs::SetBool::Response& res) {
    std::lock_guard<std::mutex> lock(mutex_);
    // 正常关闭回到观察，停止请求未完成时 ready() 仍拒绝放行；不会自动恢复 ACTIVE。
    // 停止失败后，再次明确调用 enable(false) 才重试一次，不自动重试。
    if(!req.data) {
      if((mode_=="ACTIVE" || stop_failed_) && !stop_requested_ && !stop_in_progress_) {
        if(mode_=="ACTIVE") {mode_="OBSERVE";reason_="人工关闭网关，等待停止回执";}
        stop_requested_=true;
      }
      res.success=true;res.message="网关已关闭；若此前活动或停止失败，已请求停止，完成情况请查看状态";return true;
    }
    if(mode_=="FAULT_LATCHED") {res.message="reset_required_after_site_stop";return true;}
    if(mode_=="ACTIVE") {res.success=true;res.message="already_active";return true;}
    std::string why;
    if(!ready(why)) {res.message=why;return true;}
    // 激活时创建新的时间代次；队列中早于该时刻的旧候选不能被补发到真机。
    mode_="ACTIVE";epoch_=ros::Time::now();last_stamp_=epoch_.toSec();
    last_candidate_wall_=ros::WallTime::now();
    reason_="explicit_activation";res.success=true;res.message="ACTIVE";return true;
  }
  // 人工复位只清网关锁存，不证明机械臂已经静止，也不重新打开硬件输出。
  bool reset(std_srvs::Trigger::Request&,std_srvs::Trigger::Response& res) {
    std::lock_guard<std::mutex> lock(mutex_);
    if(mode_=="ACTIVE" || stop_in_progress_ || stop_requested_) {
      res.message="close_gate_and_wait_for_stop_request_first";return true;
    }
    if(stop_failed_) {
      res.message="停止服务上次调用失败；请先处理现场停止问题，再显式调用 enable(false) 重试";return true;
    }
    mode_="OBSERVE";reason_="manual_reset";last_stamp_=0;
    res.success=true;res.message="OBSERVE; reset_does_not_enable_hardware";return true;
  }
  // 墙钟周期监督：即使没有新候选回调，也会检查输入时效和候选发布中断。
  void tick(const ros::WallTimerEvent&) {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto now=ros::Time::now();
    // ROS 时间回退会破坏先后关系；清空所有有效标志，并在运动中触发锁存。
    if(!last_ros_.isZero() && now<last_ros_) {
      joints_.valid=base_.valid=target_.valid=controller_.valid=hardware_.valid=qp_.valid=false;
      fault("ros_clock_backwards");
    }
    last_ros_=now;
    std::string why;
    const bool ok=ready(why);
    if(mode_=="ACTIVE") {
      if(!ok) fault(why);
      // 候选自身的看门狗独立于输入新鲜度；输入仍在更新但求解节点停发也必须关闭网关。
      else if((ros::WallTime::now()-last_candidate_wall_).toSec()>candidate_timeout_)
        fault("candidate_watchdog");
    // 只有非故障的观察/就绪状态可以自动互转；此分支绝不会自动进入 ACTIVE。
    } else if(mode_!="FAULT_LATCHED") {
      mode_=ok?"READY":"OBSERVE";reason_=ok?"ready_for_explicit_activation":why;
    }
    std_msgs::String state;state.data=mode_+": "+reason_;state_out_.publish(state);
  }
  // 检查 ROS master 当前登记的发布者快照；这不是 ROS1 的访问控制或持续独占锁。
  bool onlyPublisher(XmlRpc::XmlRpcValue& publishers,const std::string& name,
                     const std::string& node) {
    for(int i=0;i<publishers.size();++i) {
      if(static_cast<std::string>(publishers[i][0])==name)
        return publishers[i][1].size()==1 &&
               static_cast<std::string>(publishers[i][1][0])==node;
    }
    return false;
  }
  // 独立轮询线程执行控制器服务和 master 查询，网络等待不会占住状态互斥锁。
  // 结果带墙钟时间，由 ready() 判断是否过期；异常按不满足放行处理。
  void poll() {
    while(!quitting_ && ros::ok()) {
      try {
      controller_manager_msgs::ListControllers call;
      bool ok=list_client_.call(call),found=false;
      if(ok) for(const auto& c:call.response.controller) if(c.name==controller_name_) {
        std::set<std::string> names;
        // 核对运行控制器实际占用 PositionJointInterface 和准确六轴资源，不能只核对名称。
        bool interface_ok=!c.claimed_resources.empty();
        for(const auto& resources:c.claimed_resources) {
          interface_ok=interface_ok &&
              resources.hardware_interface=="hardware_interface::PositionJointInterface";
          names.insert(resources.resources.begin(),resources.resources.end());
        }
        found=c.state=="running" && c.type==controller_type_ && interface_ok &&
              names==std::set<std::string>(limits_.names.begin(),limits_.names.end());
      }
      XmlRpc::XmlRpcValue args,result,payload;
      // XML-RPC 的调用者身份；随后从系统图检查真实命令和候选命令的发布者。
      args[0]=ros::this_node::getName();
      const bool graph=ros::master::execute("getSystemState",args,result,payload,false);
      bool ownership=false;
      if(graph && payload.getType()==XmlRpc::XmlRpcValue::TypeArray && payload.size()==3) {
        ownership=onlyPublisher(payload[0],command_topic_,ros::this_node::getName()) &&
                  onlyPublisher(payload[0],candidate_topic_,candidate_node_);
        // Action goals bypass the command topic. Any registered goal publisher blocks ACTIVE.
        // action goal 可以绕过 command 话题；存在此类发布者时同样阻止放行。
        // 轮询有时间窗口，仍须现场管理命令源，不能把它当作安全权限机制。
        const auto action=command_topic_.substr(0,command_topic_.find_last_of('/'))+
                          "/follow_joint_trajectory/goal";
        for(int i=0;i<payload[0].size();++i)
          if(static_cast<std::string>(payload[0][i][0])==action &&
             payload[0][i][1].size()>0) ownership=false;
      }
      const bool stop_exists=stop_client_.exists();
      {std::lock_guard<std::mutex> lock(mutex_);
       poll_ok_=ok&&found&&ownership&&stop_exists;poll_wall_=ros::WallTime::now();}
      } catch(...) {
        std::lock_guard<std::mutex> lock(mutex_);
        poll_ok_=false;poll_wall_=ros::WallTime::now();
      }
      ros::WallDuration(0.1).sleep();
    }
  }
  // 独立停止线程取走一次请求并调用站点 Trigger 服务。
  // 失败回执保持故障锁存并阻止复位；成功回执仍不能替代现场物理停止确认。
  void stopLoop() {
    while(!quitting_ && ros::ok()) {
      bool requested=false;
      {std::lock_guard<std::mutex> lock(mutex_);
       // 与置 stop_in_progress_ 一起放在锁内，避免 reset 把请求交接间隙误认为已结束。
       requested=stop_requested_.exchange(false);
       if(requested)stop_in_progress_=true;}
      if(requested) {
        std_srvs::Trigger call;
        // 必须同时满足服务通信成功和服务业务 success；任何一个失败都明确报告。
        const bool ok=stop_client_.call(call) && call.response.success;
        {std::lock_guard<std::mutex> lock(mutex_);
         stop_failed_=!ok;
         if(!ok) mode_="FAULT_LATCHED";
         reason_+=ok?"; stop_service_ack":"; STOP_SERVICE_FAILED";
         stop_in_progress_=false;}
        if(!ok) ROS_ERROR("Site stop failed; independent hardware stop/watchdog required");
      }
      ros::WallDuration(0.01).sleep();
    }
  }
public:
  // 启动时一次性读取配置、校验模型限值并连接 ROS 接口。
  // 构造函数从不把 mode_ 设置为 ACTIVE，重启后必须重新走显式放行。
  RealBridge() {
    p_.param("input_timeout",timeout_,0.3);p_.param("max_input_skew",max_skew_,0.08);
    p_.param("candidate_max_age",max_candidate_age_,0.02);
    p_.param("candidate_timeout",candidate_timeout_,0.15);
    p_.param("site_review_complete",reviewed_,false);
    // Deliberately no simulation-time override: this executable is a hardware gate.
    p_.param<std::string>("world_frame",world_,"world");
    p_.param<std::string>("controller_name",controller_name_,"elfin_arm_controller");
    p_.param<std::string>("controller_type",controller_type_,"position_controllers/JointTrajectoryController");
    p_.param<std::string>("hardware_status_name",hardware_name_,"elfin5_hardware");
    p_.param<std::string>("hardware_id",hardware_id_,"");
    p_.getParam("joint_names",limits_.names);p_.getParam("position_lower",limits_.lower);
    p_.getParam("position_upper",limits_.upper);p_.getParam("velocity_limits",limits_.velocity);
    p_.getParam("acceleration_limits",limits_.acceleration);p_.getParam("max_endpoint_step",limits_.step);
    p_.param("min_horizon",limits_.min_horizon,0.02);p_.param("max_horizon",limits_.max_horizon,0.15);
    // 时效配置是放行契约的一部分；NaN 或非正阈值在启动阶段直接报错。
    for(double value:{timeout_,max_skew_,max_candidate_age_,candidate_timeout_})
      if(!std::isfinite(value)||value<=0) throw std::runtime_error("Invalid timing parameter");
    if(limits_.names.size()!=6 || std::set<std::string>(limits_.names.begin(),limits_.names.end()).size()!=6)
      throw std::runtime_error("Exactly six unique joint_names required");
    std::string why;
    model_valid_=limits_.valid(why);
    // 把站点限值与当前命名空间的 URDF 比较；位置和速度不得超出模型限制。
    urdf::Model model;
    std::string xml;
    n_.getParam("robot_description",xml);
    model_valid_=model.initString(xml) && model_valid_;
    // 当前实现只接受具有明确限位的转动关节，不自动按连续关节规则折叠角度。
    if(model_valid_) for(std::size_t i=0;i<6;++i) {
      const auto j=model.getJoint(limits_.names[i]);
      model_valid_=model_valid_ && j && j->type==urdf::Joint::REVOLUTE && j->limits;
      if(!model_valid_) break;
      model_valid_=limits_.lower[i]>=j->limits->lower && limits_.upper[i]<=j->limits->upper &&
                   limits_.velocity[i]<=j->limits->velocity;
    }
    if(!model_valid_) ROS_WARN("Site limits absent/invalid: OBSERVE only");
    // 先解析完整 ROS 名称，再核对候选/实际通道不能相同，避免命名空间掩盖重接线。
    command_topic_=n_.resolveName(topic("command_topic","/elfin_arm_controller/command"));
    candidate_topic_=n_.resolveName(topic("candidate_topic","/elfin5_real/candidate_trajectory"));
    candidate_node_=topic("candidate_node","/elfin5_real/qp_candidate");
    if(command_topic_==candidate_topic_) throw std::runtime_error("Candidate must be isolated");
    // 输出队列为1且不锁存，避免新订阅者自动收到上一轮历史命令。
    command_out_=n_.advertise<trajectory_msgs::JointTrajectory>(command_topic_,1,false);
    // validated/* 专供候选 QP 和真实关节模型显示；只有格式及时间有效的输入才转发。
    joint_out_=n_.advertise<sensor_msgs::JointState>("validated/joint_states",1);
    base_out_=n_.advertise<geometry_msgs::PoseStamped>("validated/base_pose",1);
    target_out_=n_.advertise<geometry_msgs::PointStamped>("validated/target_point",1);
    state_out_=p_.advertise<std_msgs::String>("state",1,true);
    joint_in_=n_.subscribe(topic("joint_states_topic","/joint_states"),1,&RealBridge::onJoint,this);
    base_in_=n_.subscribe(topic("base_pose_topic","/base_pose"),1,&RealBridge::onBase,this);
    target_in_=n_.subscribe(topic("target_point_topic","/target_point"),1,&RealBridge::onTarget,this);
    command_in_=n_.subscribe(candidate_topic_,1,&RealBridge::onCommand,this);
    controller_in_=n_.subscribe(topic("controller_state_topic","/elfin_arm_controller/state"),1,&RealBridge::onController,this);
    hardware_in_=n_.subscribe(topic("hardware_status_topic","/elfin5_hardware/status"),1,&RealBridge::onHardware,this);
    qp_in_=n_.subscribe("qp_candidate/state",1,&RealBridge::onQP,this);
    // 私有服务位于 bridge/enable 与 bridge/reset；与 qp_candidate/enable 的职责不同。
    enable_=p_.advertiseService("enable",&RealBridge::enable,this);
    reset_=p_.advertiseService("reset",&RealBridge::reset,this);
    list_client_=n_.serviceClient<controller_manager_msgs::ListControllers>(
        topic("list_controllers_service","/controller_manager/list_controllers"));
    stop_client_=n_.serviceClient<std_srvs::Trigger>(
        topic("stop_service","/elfin5_site/stop"));
    // 10 ms 墙钟监督定时器与两个工作线程分工；线程只在完成成员初始化后启动。
    timer_=n_.createWallTimer(ros::WallDuration(0.01),&RealBridge::tick,this);
    poll_thread_=std::thread(&RealBridge::poll,this);
    stop_thread_=std::thread(&RealBridge::stopLoop,this);
  }
  // 退出时关闭客户端并回收工作线程；析构不是已经验证的物理停车动作。
  ~RealBridge() {
    quitting_=true;list_client_.shutdown();stop_client_.shutdown();
    if(poll_thread_.joinable())poll_thread_.join();
    if(stop_thread_.joinable())stop_thread_.join();
  }
};
// ROS 入口：创建对象后开启多回调线程；共享状态由类内 mutex_ 保护。
// 初始化失败以非零退出码结束，交给 launch 的 required=true 停止整组节点。
int main(int argc,char** argv) {
  ros::init(argc,argv,"real_robot_bridge");
  try { RealBridge bridge;ros::AsyncSpinner spinner(3);spinner.start();ros::waitForShutdown(); }
  catch(const std::exception& e) {ROS_FATAL_STREAM(e.what());return 1;}
  return 0;
}
