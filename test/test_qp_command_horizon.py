#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""编译实际QP参数检查、轨迹发布和速度边界方法；ROS通信用桩，不运行MoveIt/设备。"""
import argparse
from pathlib import Path
import re
import subprocess
import tempfile

STUB = r'''
#include <Eigen/Core>
#include "elfin5_qp_controller/real_robot_command_gate.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>
namespace ros {
struct Duration {double value;explicit Duration(double v):value(v){}double toSec()const{return value;}};
struct Time {double value=0;static Time now(){Time t;t.value=12;return t;}double toSec()const{return value;}};
struct NodeHandle {
 std::map<std::string,double> values;
 template<class T>void apply(const std::string&,T&)const{}
 void apply(const std::string& k,double& v)const{auto i=values.find(k);if(i!=values.end())v=i->second;}
 template<class T>void param(const std::string& k,T& out,const T& fallback)const{out=fallback;apply(k,out);}
};
}
namespace trajectory_msgs {
struct JointTrajectoryPoint {std::vector<double> positions,velocities;ros::Duration time_from_start{0};};
struct JointTrajectory {struct Header{ros::Time stamp;}header;std::vector<std::string> joint_names;std::vector<JointTrajectoryPoint> points;};
}
struct Publisher {trajectory_msgs::JointTrajectory last;int count=0;void publish(const trajectory_msgs::JointTrajectory& m){last=m;++count;}};
'''

HARNESS = r'''
int count=0;
void check(bool ok,const char* why){++count;if(!ok)throw std::runtime_error(why);}
bool near(double a,double b){return std::abs(a-b)<1e-12;}
int main(){try{
 using elfin5_qp_controller::QPCartesianController;
 QPCartesianController c;c.loadParameters();
 check(near(c.command_horizon_,.14),"默认固定140ms");
 check(near(c.control_rate_,50) && near(c.goal_tolerance_,.005) && c.qp_max_iterations_==100,"频率容差与预算不变");
 c.private_node_handle_.values["command_horizon"]=.12;c.loadParameters();
 check(near(c.command_horizon_,.12),"读取秒制参数");
 for(double v:{0.,-.1,std::numeric_limits<double>::infinity(),std::numeric_limits<double>::quiet_NaN()}){
   c.private_node_handle_.values["command_horizon"]=v;bool rejected=false;
   try{c.loadParameters();}catch(const std::invalid_argument&){rejected=true;}check(rejected,"非法时域启动拒绝");
 }
 c.private_node_handle_.values.clear();c.loadParameters();
 for(int i=1;i<=6;++i)c.joint_names_.push_back("elfin_joint"+std::to_string(i));
 c.minimum_positions_=Eigen::VectorXd::Constant(6,-2.);
 c.maximum_positions_=Eigen::VectorXd::Constant(6,2.);
 c.maximum_velocities_=Eigen::VectorXd::Constant(6,1.57);
 c.maximum_accelerations_=Eigen::VectorXd::Constant(6,12.56);
 c.velocity_limit_scale_=.05;c.acceleration_limit_scale_=.02;
 Eigen::VectorXd q=Eigen::VectorXd::Constant(6,1.),v=Eigen::VectorXd::Constant(6,.03);
 for(double dt:{.01,.02,.03,.2}){
   c.control_rate_=1./dt;c.moveForTest(q,v);
   const auto& m=c.command_publisher_.last;
   check(m.points.size()==1 && near(m.points[0].time_from_start.toSec(),.14),"周期变化不能改变时域");
   check(m.joint_names==c.joint_names_ && near(m.header.stamp.toSec(),12),"名称和生成时间戳");
   for(int i=0;i<6;++i){check(near(m.points[0].positions[i],1.+.14*.03),"端点用同一140ms预测");check(near(m.points[0].velocities[i],.03),"端点速度原样保留");}
   c.previous_velocity_command_=v;c.publishZeroVelocityHold(q);
   const auto& hold=c.command_publisher_.last.points[0];
   check(near(hold.time_from_start.toSec(),.14),"QP保持也固定140ms");
   check(c.previous_velocity_command_.isZero(),"保持重置上一QP速度");
   for(int i=0;i<6;++i)check(near(hold.positions[i],1.) && near(hold.velocities[i],0),"保持用实测位置和零端点速度");
 }
 Eigen::VectorXd lo,hi;c.previous_velocity_command_=v;
 for(double dt:{.01,.02,.03}){
   check(c.buildVelocityBounds(q,dt,c.command_horizon_,lo,hi),"常规速度约束可行");
   for(int i=0;i<6;++i){check(near(lo[i],.03-.2512*dt),"加速度下界仍用dt");check(near(hi[i],.03+.2512*dt),"加速度上界仍用dt");}
 }
 q.setConstant(1.966);check(c.buildVelocityBounds(q,.02,c.command_horizon_,lo,hi),"接近限位可行");
 for(int i=0;i<6;++i)check(near(hi[i],(2.-.03-1.966)/.14),"位置预测边界使用140ms");
 q.setConstant(1.9699);check(!c.buildVelocityBounds(q,.02,c.command_horizon_,lo,hi),"位置与加速度冲突不绕过");
 namespace eq=elfin5_qp_controller;eq::HardwareLimits l;l.names=c.joint_names_;
 l.lower.assign(6,-2);l.upper.assign(6,2);l.velocity.assign(6,.0785);l.acceleration.assign(6,.2512);l.step.assign(6,.02);
 eq::HardwareCommand cmd;cmd.names=l.names;cmd.stamp=10;cmd.horizon=.14;cmd.positions.assign(6,.0014);cmd.velocities.assign(6,.01);
 std::vector<double> actual(6,0),reference(6,0),refv(6,.01);std::string why;
 check(eq::checkCommand(l,cmd,actual,reference,refv,10.005,9,9,.02,why),"140ms在现有网关时域内且样例包络通过");
 cmd.horizon=.16;check(!eq::checkCommand(l,cmd,actual,reference,refv,10.005,9,9,.02,why) && why=="invalid_command_horizon","不放宽150ms网关上限");
 std::cout<<"通过 "<<count<<" 项实际QP方法/网关断言；ROS接口桩，未验证ROS/MoveIt/Gazebo/真机\n";
 return 0;
 }catch(const std::exception& e){std::cerr<<e.what()<<"\n";return 1;}}
'''


def method(source, name):
    marker = 'QPCartesianController::'+name+'('
    start = source.rfind('\n',0,source.index(marker))+1
    brace = source.index('{',source.index(marker))
    depth=1;end=brace+1
    while depth:
        depth += (source[end]=='{')-(source[end]=='}')
        end+=1
    return source[start:end]


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--zig')
    parser.add_argument('--eigen',default='/usr/include/eigen3')
    args=parser.parse_args()
    root=Path(__file__).resolve().parents[1]
    source=(root/'src_gazebo/qp_cartesian_controller.cpp').read_text(encoding='utf-8')
    header=(root/'include/elfin5_qp_controller/qp_cartesian_controller.h').read_text(encoding='utf-8')
    config=(root/'config/elfin5_qp_controller.yaml').read_text(encoding='utf-8')
    assert re.search(r'^command_horizon:\s*0\.14\s*$',config,re.M)
    assert 'command_horizon_factor' not in source+header+config
    fixed=re.search(r'const double command_horizon = [^;]+;',source).group()
    assert fixed=='const double command_horizon = command_horizon_;'
    assert 'buildVelocityBounds(positions, dt, command_horizon,' in source
    params=source[source.index('  private_node_handle_.param<std::string>'):source.index('  // 求解器参数由YAML')]
    fields='\n'.join(t+' '+n+';' for t,n in re.findall(r'^\s+(double|int|std::string) (\w+_);',header,re.M))
    movement=source[source.index('  previous_velocity_command_ = solution.x;'):source.index('\n\n',source.index('  previous_velocity_command_ = solution.x;'))]
    declarations=r'''
namespace elfin5_qp_controller {
class QPCartesianController {public:
 ros::NodeHandle private_node_handle_;Publisher command_publisher_;
 std::vector<std::string> joint_names_;
 Eigen::VectorXd minimum_positions_,maximum_positions_,maximum_velocities_,maximum_accelerations_,previous_velocity_command_;
 FIELDS
 void loadParameters(){PARAMS}
 bool buildVelocityBounds(const Eigen::VectorXd&,double,double,Eigen::VectorXd&,Eigen::VectorXd&)const;
 void publishTrajectoryCommand(const Eigen::VectorXd&,const Eigen::VectorXd&,double);
 void publishZeroVelocityHold(const Eigen::VectorXd&);
 void moveForTest(const Eigen::VectorXd& positions,const Eigen::VectorXd& velocity){
  struct Solution {Eigen::VectorXd x;}solution{velocity};FIXED
  MOVEMENT
 }
};
'''.replace('FIELDS',fields).replace('PARAMS',params).replace('FIXED',fixed).replace('MOVEMENT',movement)
    body=STUB+declarations+'\n'.join(method(source,n) for n in
        ['buildVelocityBounds','publishTrajectoryCommand','publishZeroVelocityHold'])+'\n}\n'+HARNESS
    with tempfile.TemporaryDirectory(prefix='elfin5_qp_horizon_') as temp:
        path=Path(temp);cpp=path/'horizon.cpp';exe=path/'horizon.exe';cpp.write_text(body,encoding='utf-8')
        compiler=[args.zig,'c++'] if args.zig else ['g++']
        flags=['-fdelayed-template-parsing'] if args.zig else []
        subprocess.check_call(compiler+['-std=c++14','-O2','-Wall','-Wextra','-Wpedantic']+flags+
            ['-I',str(root/'include'),'-isystem',args.eigen,str(cpp),str(root/'src/real_robot_command_gate.cpp'),'-o',str(exe)])
        subprocess.check_call([str(exe)],timeout=15)


if __name__=='__main__':main()
