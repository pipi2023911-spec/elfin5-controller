#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""开发机C++回调测试：编译实际网关方法，ROS类型/时钟/服务用桩替代。

不执行生产构造函数，不验证ROS ABI、真实接线、调度或停车。
用法：python3 test/test_bridge_callbacks.py [--zig 编译器路径]
"""
import argparse
import pathlib
import subprocess
import tempfile

STUB = r'''
#pragma once
#include <memory>
#include <string>
#include <vector>
#include <functional>
#include <typeinfo>
#include <iostream>
namespace ros {
static double clock_now=10; static int loop_budget=0;
struct Duration { double value; Duration(double v=0):value(v){} double toSec()const{return value;} };
using WallDuration=Duration;
struct Time {
 double value; Time(double v=0):value(v){} static Time now(){return Time(clock_now);}
 static bool isSimTime(){return false;} double toSec()const{return value;}
 void fromSec(double v){value=v;} bool isZero()const{return value==0;}
};
inline bool operator>(Time a,Time b){return a.value>b.value;}
inline bool operator<(Time a,Time b){return a.value<b.value;}
inline Duration operator-(Time a,Time b){return Duration(a.value-b.value);}
using SteadyTime=Time;
struct WallTimerEvent{};
struct WallTimer{}; struct Subscriber{}; struct ServiceServer{};
static std::function<void(const std::type_info&,const void*)> sink;
static std::function<bool(const std::type_info&,void*)> service;
struct Publisher {
 template<class T> void publish(const T& t){if(sink)sink(typeid(T),&t);}
 template<class T> void publish(const std::shared_ptr<const T>& t){publish(*t);}
};
struct ServiceClient {
 template<class T> bool call(T& t){return service?service(typeid(T),&t):false;}
 bool exists(){return true;} void shutdown(){}
};
struct NodeHandle {
 NodeHandle(const char* = ""){}
 template<class T> void param(const std::string&,T& v,const T& fallback){v=fallback;}
 template<class T> bool getParam(const std::string&,T&){return false;}
 std::string resolveName(const std::string& s){return s;}
 template<class T> Publisher advertise(const std::string&,int,bool=false){return Publisher();}
 template<class F,class C> Subscriber subscribe(const std::string&,int,F,C*){return Subscriber();}
 template<class F,class C> ServiceServer advertiseService(const std::string&,F,C*){return ServiceServer();}
 template<class T> ServiceClient serviceClient(const std::string&){return ServiceClient();}
 template<class F,class C> WallTimer createWallTimer(Duration,F,C*){return WallTimer();}
};
inline bool ok(){return loop_budget-->0;}
inline void init(int,char**,const char*){}
inline void waitForShutdown(){}
struct AsyncSpinner {AsyncSpinner(int){} void start(){} };
}
#define ROS_ERROR_STREAM(x) do{}while(false)
#define ROS_FATAL_STREAM(x) do{}while(false)
#define ROS_WARN(x) do{}while(false)
namespace std_msgs {
struct Header {ros::Time stamp;};
struct Bool {using ConstPtr=std::shared_ptr<const Bool>;bool data=false;};
struct String {std::string data;};
}
namespace sensor_msgs {
struct JointState {using ConstPtr=std::shared_ptr<const JointState>;
 std_msgs::Header header;std::vector<std::string> name;std::vector<double> position;};
}
namespace trajectory_msgs {
struct JointTrajectoryPoint {std::vector<double> positions,velocities,accelerations,effort;ros::Duration time_from_start;};
struct JointTrajectory {using ConstPtr=std::shared_ptr<const JointTrajectory>;
 std_msgs::Header header;std::vector<std::string> joint_names;std::vector<JointTrajectoryPoint> points;};
}
namespace control_msgs {
struct JointTrajectoryControllerState {using ConstPtr=std::shared_ptr<const JointTrajectoryControllerState>;
 std_msgs::Header header;std::vector<std::string> joint_names;
 trajectory_msgs::JointTrajectoryPoint actual,desired;};
}
namespace controller_manager_msgs {
struct Resource {std::string hardware_interface;std::vector<std::string> resources;};
struct Controller {std::string name,state,type;std::vector<Resource> claimed_resources;};
struct ListControllers {struct Response {std::vector<Controller> controller;} response;};
}
namespace std_srvs {
struct SetBool {struct Request{bool data=false;};struct Response{bool success=false;std::string message;};};
struct Trigger {struct Request{};struct Response{bool success=false;std::string message;} response;};
}
namespace urdf {
struct Limits {double lower=-2,upper=2,velocity=1;};
struct Joint {enum {REVOLUTE=1};int type=REVOLUTE;std::shared_ptr<Limits> limits{new Limits};};
struct Model {bool initString(const std::string&){return true;}
 std::shared_ptr<Joint> getJoint(const std::string&){return std::make_shared<Joint>();}};
}
'''
# sleep不是被测业务；用一个无等待派生替换桩Duration定义。
STUB = STUB.replace('double toSec()const{return value;} };',
                    'double toSec()const{return value;} void sleep()const{} };', 1)

HARNESS = r'''
static int count=0,stops=0;static bool stop_success=true;
static std::vector<trajectory_msgs::JointTrajectory> sent;
void check(bool v,const char* why){++count;if(!v)throw std::runtime_error(why);}
void setup(RealBridge& b) {
 b.timeout_=.3;b.max_skew_=.08;b.max_age_=.02;b.candidate_timeout_=.15;b.hold_horizon_=.1;
 b.reviewed_=b.model_valid_=true;b.driver_enabled_=true;b.driver_fault_=false;
 b.controller_name_="elfin_arm_controller";b.controller_type_="position_controllers/JointTrajectoryController";
 for(int i=1;i<=6;++i)b.limits_.names.push_back("j"+std::to_string(i));
 b.limits_.lower.assign(6,-2);b.limits_.upper.assign(6,2);b.limits_.velocity.assign(6,1);
 b.limits_.acceleration.assign(6,10);b.limits_.step.assign(6,.1);
 b.safe_pose_.assign(6,.02);b.reference_=b.safe_pose_;b.reference_velocity_.assign(6,0);
 b.joints_.set(ros::Time(9.998));b.controller_.set(ros::Time(9.998));
 b.enabled_age_.set(ros::Time(10));b.fault_age_.set(ros::Time(10));
 ros::clock_now=10;sent.clear();stops=0;stop_success=true;
}
std::shared_ptr<trajectory_msgs::JointTrajectory> candidate(RealBridge& b) {
 auto c=std::make_shared<trajectory_msgs::JointTrajectory>();c->header.stamp=ros::Time(9.999);
 c->joint_names=b.limits_.names;trajectory_msgs::JointTrajectoryPoint p;
 p.positions.assign(6,.021);p.velocities.assign(6,0);p.time_from_start=ros::Duration(.1);c->points.push_back(p);return c;
}
void active(RealBridge& b){b.mode_="ACTIVE";b.epoch_=ros::Time(9);b.last_stamp_=9;b.last_candidate_=ros::Time(10);}
void finishStop(RealBridge& b){ros::loop_budget=1;b.stopLoop();}
int main(){
 ros::sink=[](const std::type_info& t,const void* p){
  if(t==typeid(trajectory_msgs::JointTrajectory))sent.push_back(*static_cast<const trajectory_msgs::JointTrajectory*>(p));};
 ros::service=[](const std::type_info& t,void* p){
  if(t==typeid(std_srvs::Trigger)){++stops;static_cast<std_srvs::Trigger*>(p)->response.success=stop_success;return true;}
  auto& r=static_cast<controller_manager_msgs::ListControllers*>(p)->response;
  controller_manager_msgs::Controller c;c.name="elfin_arm_controller";c.state="running";
  c.type="position_controllers/JointTrajectoryController";controller_manager_msgs::Resource a;
  a.hardware_interface="hardware_interface::PositionJointInterface";
  for(int i=1;i<=6;++i)a.resources.push_back("j"+std::to_string(i));c.claimed_resources.push_back(a);r.controller.push_back(c);return true;};
 try {
  {RealBridge b;setup(b);auto c=candidate(b);b.onCommand(c);check(sent.empty(),"未放行不能输出");
   std_srvs::SetBool::Request req;req.data=true;std_srvs::SetBool::Response res;b.enable(req,res);
   check(res.success && b.mode_=="ACTIVE","人工放行");
   c->header.stamp=ros::Time(10.001);ros::clock_now=10.002;b.onCommand(c);
   if(sent.size()!=1)std::cerr<<b.reason_<<"，输出数="<<sent.size()<<"\n";
   check(sent.size()==1,"正常候选转发");
   c->header.stamp=ros::Time(10.002);c->points[0].positions[0]=NAN;ros::clock_now=10.003;b.onCommand(c);
   check(b.mode_=="FAULT_LATCHED" && b.stop_pending_,"非法候选锁存并请求停止");
   check(sent.size()==2 && sent.back().points[0].positions==b.safe_pose_,"保持使用实际反馈而非候选");
   check(sent.back().points[0].velocities==std::vector<double>(6,0),"保持端点零速");
   b.onCommand(c);check(sent.size()==2,"不重复保持或透传坏命令");finishStop(b);check(stops==1,"一次站点停止调用");
   std_srvs::Trigger::Request rr;std_srvs::Trigger::Response rs;b.reset(rr,rs);
   check(rs.success && b.mode_=="OBSERVE","复位只关闭不恢复");b.onCommand(c);check(sent.size()==2,"复位后仍不输出");}
  {RealBridge b;ros::clock_now=10;setup(b);active(b);ros::clock_now=10.4;b.tick(ros::WallTimerEvent());
   check(b.mode_=="FAULT_LATCHED" && sent.empty(),"反馈超时不得向旧姿态回拉");finishStop(b);check(stops==1,"反馈超时调用站点停止保持");}
  {RealBridge b;ros::clock_now=10;setup(b);active(b);auto f=std::make_shared<std_msgs::Bool>();f->data=true;b.onDriverFault(f);
   check(b.mode_=="FAULT_LATCHED" && sent.empty(),"驱动故障不得继续发送位置保持");
   stop_success=false;finishStop(b);std_srvs::Trigger::Request q;std_srvs::Trigger::Response r;b.reset(q,r);
   check(!r.success,"停止失败阻止复位");stop_success=true;std_srvs::SetBool::Request off;off.data=false;std_srvs::SetBool::Response out;
   b.enable(off,out);check(b.stop_pending_,"人工关闭明确重试停止");finishStop(b);check(stops==2 && !b.stop_failed_,"重试应答不自动恢复运动");}
  {RealBridge b;ros::clock_now=10;setup(b);active(b);std_srvs::SetBool::Request off;std_srvs::SetBool::Response out;b.enable(off,out);
   check(out.success && b.mode_=="FAULT_LATCHED" && sent.size()==1,"人工关闭也请求一次保持并锁存");}
  {RealBridge b;ros::clock_now=10;setup(b);active(b);
   auto j=std::make_shared<sensor_msgs::JointState>();j->header.stamp=ros::Time(9.999);
   j->name=b.limits_.names;j->position.assign(6,.03);b.onJoint(j);
   check(b.safe_pose_==j->position,"缓存更新自真实反馈回调");
   auto c=candidate(b);c->points[0].positions[0]=NAN;b.onCommand(c);
   check(sent.size()==1 && sent.back().points[0].positions==j->position,"故障保持冻结最新有效实际姿态");
   j->header.stamp=ros::Time(10);j->position.assign(6,.031);b.onJoint(j);
   check(sent.size()==1 && sent.back().points[0].positions[0]==.03,"故障后更新反馈不能重发漂移保持目标");}
  {RealBridge b;ros::clock_now=10;setup(b);active(b);b.last_candidate_=ros::Time(9.8);
   b.tick(ros::WallTimerEvent());check(b.mode_=="FAULT_LATCHED" && sent.size()==1,"候选超时但反馈有效时一次保持");}
  {RealBridge b;ros::clock_now=10;setup(b);active(b);auto off=std::make_shared<std_msgs::Bool>();b.onEnabled(off);
   check(b.mode_=="FAULT_LATCHED" && sent.empty(),"驱动失能时不继续发保持位置");}
  {RealBridge b;ros::clock_now=10;setup(b);auto original=ros::service;
   ros::service=[&b,original](const std::type_info& t,void* p){
    if(t==typeid(controller_manager_msgs::ListControllers)){
     std_srvs::SetBool::Request off;std_srvs::SetBool::Response r;b.enable(off,r);}
    return original(t,p);};
   std_srvs::SetBool::Request on;on.data=true;std_srvs::SetBool::Response r;b.enable(on,r);
   check(!r.success && b.mode_=="OBSERVE","查询期间人工关闭应使旧放行请求失效");ros::service=original;}
  std::cout<<"通过 "<<count<<" 项实际网关方法桩测试；未验证ROS ABI/现场停车\n";return 0;
 }catch(const std::exception& e){std::cerr<<e.what()<<"\n";return 1;}
}
'''

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--zig', help='可选的Zig可执行文件，默认使用g++')
    args = parser.parse_args()
    root = pathlib.Path(__file__).resolve().parents[1]
    source = (root/'src/real_robot_bridge_node.cpp').read_text(encoding='utf-8')
    # 方法体保持原样；只公开成员、跳过依赖ROS注册的构造、重命名生产入口。
    source = source.replace('class RealBridge {', 'class RealBridge {\npublic:', 1)
    source = source.replace('  RealBridge() {', '  void unusedRegistration() {', 1)
    source = source.replace('int main(int argc,char** argv)', 'int unusedMain(int argc,char** argv)', 1)
    headers = ['ros/ros.h','sensor_msgs/JointState.h','trajectory_msgs/JointTrajectory.h',
               'control_msgs/JointTrajectoryControllerState.h','controller_manager_msgs/ListControllers.h',
               'std_msgs/Bool.h','std_msgs/String.h','std_srvs/SetBool.h','std_srvs/Trigger.h','urdf/model.h']
    with tempfile.TemporaryDirectory(prefix='elfin5_callbacks_') as temp:
        path = pathlib.Path(temp)
        (path/'stub.hpp').write_text(STUB, encoding='utf-8')
        for header in headers:
            target=path/header; target.parent.mkdir(parents=True,exist_ok=True)
            target.write_text('#include "stub.hpp"\n',encoding='utf-8')
        (path/'callbacks.cpp').write_text(source+'\n'+HARNESS,encoding='utf-8')
        exe=path/'callbacks.exe'
        compiler=[args.zig,'c++'] if args.zig else ['g++']
        cmd=compiler+['-std=c++14','-pthread','-w','-I',str(path),'-I',str(root/'include'),
                      str(path/'callbacks.cpp'),str(root/'src/real_robot_command_gate.cpp'),'-o',str(exe)]
        subprocess.check_call(cmd)
        subprocess.check_call([str(exe)])

if __name__=='__main__':
    main()
