#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Windows/Linux离线编译实际stop适配器；ROS接口使用桩，不连接任何设备。"""
import argparse
from pathlib import Path
import subprocess
import tempfile

STUB = r'''
#pragma once
#include <atomic>
#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>
namespace ros {
static const auto origin=std::chrono::steady_clock::now();
static std::atomic<bool> frozen{false},running{true};
static std::atomic<double> shift{0.};
struct Duration {double v;double toSec()const{return v;}};
struct Time {
 double v=0.;Time()=default;explicit Time(double x):v(x){}
 static Time now(){return Time(frozen?100.:10.+shift.load()+std::chrono::duration<double>(std::chrono::steady_clock::now()-origin).count());}
 bool isZero()const{return v==0.;}
};
inline bool operator>(Time a,Time b){return a.v>b.v;}inline bool operator<(Time a,Time b){return a.v<b.v;}
inline Duration operator-(Time a,Time b){return Duration{a.v-b.v};}
struct Header {Time stamp;};
struct PubStats {std::atomic<int> count{0},subscribers{1};std::atomic<bool> fail{false};};
struct Publisher {
 std::shared_ptr<PubStats> stats{new PubStats};
 int getNumSubscribers()const{return stats->subscribers;}
 template<class T>void publish(const T& m){if(stats->fail)throw std::runtime_error("fake_publish_failure");if(!m.points.empty())throw std::runtime_error("nonempty_stop");++stats->count;}
};
struct Subscriber{};struct ServiceServer{};
struct NodeHandle {
 NodeHandle()=default;explicit NodeHandle(const char*){}
 template<class T>bool getParam(const std::string&,T&){return false;}
 template<class T>void param(const std::string&,T& out,const T& fallback){out=fallback;}
 template<class T>Publisher advertise(const std::string&,int,bool){return Publisher();}
 template<class... T>Subscriber subscribe(T&&...){return Subscriber();}
 template<class... T>ServiceServer advertiseService(T&&...){return ServiceServer();}
};
inline bool ok(){return running;}inline void init(int,char**,const char*){}
struct AsyncSpinner {explicit AsyncSpinner(int){}void start(){}};inline void waitForShutdown(){}
}
#define ROS_WARN(...) ((void)0)
namespace sensor_msgs {struct JointState {using ConstPtr=std::shared_ptr<const JointState>;ros::Header header;std::vector<std::string> name;std::vector<double> velocity;};}
namespace control_msgs {struct JointTrajectoryControllerState {using ConstPtr=std::shared_ptr<const JointTrajectoryControllerState>;ros::Header header;std::vector<std::string> joint_names;struct Point{std::vector<double> velocities;}desired;};}
namespace trajectory_msgs {struct JointTrajectory {std::vector<int> points;};}
namespace std_srvs {struct Trigger {struct Request{};struct Response{bool success=false;std::string message;};};}
'''

HARNESS = r'''
#include <iostream>
#include <limits>
#include <thread>
int checks=0,cases=0;
void check(bool yes,const char* why){++checks;if(!yes)throw std::runtime_error(why);}
// Windows线程唤醒可能接近15.6ms；测试阈值验证协议，不假设2ms实时调度。
void setup(StopHoldAdapter& a){a.duration_=.04;a.timeout_=.30;a.feedback_timeout_=.12;}
void sample(StopHoldAdapter& a,const std::string& mode) {
 auto j=std::make_shared<sensor_msgs::JointState>();
 auto s=std::make_shared<control_msgs::JointTrajectoryControllerState>();
 j->name=s->joint_names=a.names_;j->velocity.assign(6,0.);s->desired.velocities.assign(6,0.);
 j->header.stamp=s->header.stamp=ros::Time::now();
 if(mode=="one_joint"){j->name.resize(1);j->velocity.resize(1);}
 if(mode=="short_velocity")j->velocity.resize(5);
 if(mode=="nan")j->velocity[1]=std::numeric_limits<double>::quiet_NaN();
 if(mode=="duplicate")j->name[1]=j->name[0];
 if(mode=="moving")j->velocity[0]=.1;
 if(mode=="at_threshold")j->velocity[0]=a.threshold_;
 if(mode=="desired_moving")s->desired.velocities[0]=.1;
 if(mode=="old_stamp")j->header.stamp=s->header.stamp=ros::Time(ros::Time::now().v-1.);
 if(mode=="future_stamp")j->header.stamp=s->header.stamp=ros::Time(ros::Time::now().v+1.);
 if(mode=="zero_stamp")j->header.stamp=s->header.stamp=ros::Time();
 if(mode=="reordered"){std::reverse(j->name.begin(),j->name.end());std::reverse(s->joint_names.begin(),s->joint_names.end());}
 a.onJoint(j);if(mode!="no_state")a.onState(s);
}
std_srvs::Trigger::Response invoke(StopHoldAdapter& a){std_srvs::Trigger::Request r;std_srvs::Trigger::Response out;check(a.stop(r,out),"service_transport");return out;}
void pumpCase(const std::string& mode,bool expected) {
 StopHoldAdapter a;setup(a);check(a.command_.stats->count==0,"startup_must_not_publish");
 std::atomic<bool> done{false};
 std::thread feed([&]{while(!done){sample(a,mode);std::this_thread::sleep_for(std::chrono::milliseconds(2));}});
 const auto result=invoke(a);done=true;feed.join();
 std::cout<<mode<<": "<<(result.success?"confirmed":"rejected")<<" | "<<result.message<<std::endl;
 check(result.success==expected,mode.c_str());check(a.command_.stats->count==1,"one_empty_request");++cases;
}
int main(){
 pumpCase("fresh",true);pumpCase("reordered",true);
 {StopHoldAdapter a;std::atomic<bool> done{false};std::thread feed([&]{while(!done){sample(a,"fresh");std::this_thread::sleep_for(std::chrono::milliseconds(4));}});
  const auto out=invoke(a);done=true;feed.join();check(out.success,"production_defaults_fresh");++cases;}
 for(const std::string mode:{"one_joint","short_velocity","nan","duplicate","moving","at_threshold","desired_moving","old_stamp","future_stamp","zero_stamp","no_state"})pumpCase(mode,false);
 {StopHoldAdapter a;setup(a);sample(a,"fresh");check(!invoke(a).success,"pre_request_zero_must_not_confirm");++cases;}
 {StopHoldAdapter a;setup(a);std::thread feed([&]{while(!a.command_.stats->count)std::this_thread::yield();sample(a,"fresh");});
  const auto out=invoke(a);feed.join();check(!out.success,"single_new_zero_is_not_sustained_confirmation");++cases;}
 {StopHoldAdapter a;setup(a);a.command_.stats->subscribers=0;check(!invoke(a).success,"no_subscriber");check(a.command_.stats->count==0,"disconnected_no_publish");++cases;}
 {StopHoldAdapter a;setup(a);a.command_.stats->fail=true;check(!invoke(a).success,"publish_failure");++cases;}
 {StopHoldAdapter a;setup(a);ros::frozen=true;const auto t=std::chrono::steady_clock::now();
  check(!invoke(a).success,"frozen_ros_must_timeout");const double elapsed=std::chrono::duration<double>(std::chrono::steady_clock::now()-t).count();
  check(elapsed>=a.timeout_ && elapsed<1.,"steady_deadline");ros::frozen=false;++cases;}
 {StopHoldAdapter a;setup(a);std_srvs::Trigger::Response first;std::thread request([&]{first=invoke(a);});
  while(!a.command_.stats->count)std::this_thread::yield();const auto second=invoke(a);request.join();
  check(!second.success && second.message=="已有停止请求处理中","concurrent_request");check(a.command_.stats->count==1,"concurrent_no_republish");++cases;}
 {StopHoldAdapter a;setup(a);a.duration_=.03;std::atomic<bool> done{false};
  std::thread feed([&]{const auto start=std::chrono::steady_clock::now();while(!done){
    const double t=std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();
    sample(a,t<.008?"fresh":"moving");std::this_thread::sleep_for(std::chrono::milliseconds(2));}});
  check(!invoke(a).success,"one_quiet_burst_not_enough");done=true;feed.join();++cases;}
 {StopHoldAdapter a;setup(a);std::thread change([&]{while(!a.command_.stats->count)std::this_thread::yield();ros::shift=-2.;});
  const auto out=invoke(a);change.join();check(!out.success && out.message=="停止确认期间ROS时间回退","clock_rollback");ros::shift=0.;++cases;}
 {StopHoldAdapter a;setup(a);std::thread change([&]{while(!a.command_.stats->count)std::this_thread::yield();a.command_.stats->subscribers=0;});
  const auto out=invoke(a);change.join();check(!out.success && out.message=="停止确认期间命令通路断开","disconnect_after_publish");++cases;}
 std::cout<<"通过 "<<cases<<" 个stop场景、"<<checks<<" 项断言；仅ROS接口桩，未验证实际停车\n";
}
'''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--zig')
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    source = (root/'src/stop_hold_adapter_node.cpp').read_text(encoding='utf-8')
    source = source.replace('class StopHoldAdapter {','class StopHoldAdapter {\npublic:',1)
    source = source.replace('int main(int argc,char** argv)','int unusedMain(int argc,char** argv)',1)
    with tempfile.TemporaryDirectory(prefix='elfin5_stop_') as temp:
        path = Path(temp)
        (path/'stub.hpp').write_text(STUB,encoding='utf-8')
        for header in ['ros/ros.h','sensor_msgs/JointState.h','control_msgs/JointTrajectoryControllerState.h',
                       'trajectory_msgs/JointTrajectory.h','std_srvs/Trigger.h']:
            target = path/header
            target.parent.mkdir(parents=True,exist_ok=True)
            target.write_text('#include "stub.hpp"\n',encoding='utf-8')
        (path/'adapter.cpp').write_text(source+'\n'+HARNESS,encoding='utf-8')
        compiler = [args.zig,'c++'] if args.zig else ['g++']
        exe = path/'adapter.exe'
        subprocess.check_call(compiler+['-std=c++14','-pthread','-Wall','-Wextra','-Wpedantic',
                                        '-I',str(path),str(path/'adapter.cpp'),'-o',str(exe)])
        subprocess.check_call([str(exe)],timeout=15)


if __name__ == '__main__':
    main()
