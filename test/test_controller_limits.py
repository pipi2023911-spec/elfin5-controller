#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""开发机检查手册90%参数，并编译实际限值合并/速度边界代码；ROS参数接口用桩。

不验证ROS ABI、MoveIt加载、Gazebo、JTC插值或真机加速度。
用法：python3 test/test_controller_limits.py --eigen /usr/include/eigen3 [--zig /path/to/zig]
"""
import argparse
import ast
import math
from pathlib import Path
import re
import subprocess
import tempfile


def value(text, key):
    match = re.search(r'^\s*'+re.escape(key)+r':\s*([^#\n]+)', text, re.M)
    if not match:
        raise ValueError('缺少参数：'+key)
    return ast.literal_eval(match.group(1).strip())


HARNESS = r'''
#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>
#define ROS_ERROR(...) ((void)0)
#define ROS_INFO_STREAM(...) ((void)0)
struct Params {
  std::map<std::string,std::vector<double>> data;
  bool getParam(const std::string& key,std::vector<double>& out) {
    auto it=data.find(key); if(it==data.end()) return false; out=it->second; return true;
  }
};
class QPCartesianController {
 public:
  Params private_node_handle_;
  std::vector<std::string> joint_names_ = {"j1","j2","j3","j4","j5","j6"};
  Eigen::VectorXd minimum_positions_=Eigen::VectorXd::Constant(6,-10.);
  Eigen::VectorXd maximum_positions_=Eigen::VectorXd::Constant(6,10.);
  Eigen::VectorXd maximum_velocities_=Eigen::VectorXd::Constant(6,10.);
  Eigen::VectorXd maximum_accelerations_=Eigen::VectorXd::Constant(6,1.);
  Eigen::VectorXd previous_velocity_command_=Eigen::VectorXd::Zero(6);
  double position_limit_margin_=0.,velocity_limit_scale_=1.,acceleration_limit_scale_=1.;
  bool configure() {
    const Eigen::Index count=6;
    // ACTUAL_CONFIGURATION
    return true;
  }
  bool buildVelocityBounds(const Eigen::VectorXd&,double,double,Eigen::VectorXd&,Eigen::VectorXd&) const;
};
// ACTUAL_VELOCITY_BOUNDS
int checks=0;
void check(bool ok,const char* why) {++checks;if(!ok)throw std::runtime_error(why);}
bool near(double x,double y){return std::abs(x-y)<1e-12;}
QPCartesianController configured() {
  QPCartesianController c;
  // PARAMETERS
  return c;
}
int main() {
  auto c=configured();check(c.configure(),"wide_model_accepts_handbook_caps");
  const auto upper=c.private_node_handle_.data["joint_position_upper_limits"];
  const auto velocity=c.private_node_handle_.data["joint_velocity_limits"];
  for(int i=0;i<6;++i){check(near(c.maximum_positions_[i],upper[i]),"upper90");
    check(near(c.minimum_positions_[i],-upper[i]),"lower90");
    check(near(c.maximum_velocities_[i],velocity[i]),"velocity90");}
  const std::vector<double> old={3.14,2.35,2.61,3.14,2.56,3.14};
  c=configured();for(int i=0;i<6;++i){c.minimum_positions_[i]=-old[i];c.maximum_positions_[i]=old[i];}
  c.maximum_velocities_.setConstant(1.57);check(c.configure(),"legacy_model_intersection");
  for(int i=0;i<6;++i){check(near(c.maximum_positions_[i],std::min(old[i],upper[i])),"never_expand_urdf_position");
    check(near(c.minimum_positions_[i],-std::min(old[i],upper[i])),"never_expand_urdf_negative");
    check(near(c.maximum_velocities_[i],1.57),"never_expand_model_velocity");}
  Eigen::VectorXd lo,hi,q=Eigen::VectorXd::Zero(6);
  check(c.buildVelocityBounds(q,.02,.05,lo,hi),"20ms_bounds_feasible");
  for(int i=0;i<6;++i){check(near(lo[i],-.02)&&near(hi[i],.02),"1rad_s2_means_delta_v_002");}
  c.velocity_limit_scale_=.05;c.acceleration_limit_scale_=.25;
  check(c.buildVelocityBounds(q,.02,.05,lo,hi),"candidate_bounds_feasible");
  for(int i=0;i<6;++i)check(near(lo[i],-.005)&&near(hi[i],.005),"candidate_acceleration_025");
  c.previous_velocity_command_.setConstant(.02);
  check(c.buildVelocityBounds(q,.02,.05,lo,hi),"candidate_nonzero_history");
  check(near(lo[0],.015)&&near(hi[0],.025),"previous_command_acceleration_window");
  c=configured();check(c.configure(),"near_limit_setup");
  q=c.maximum_positions_-Eigen::VectorXd::Constant(6,.0001);c.previous_velocity_command_.setConstant(.1);
  check(!c.buildVelocityBounds(q,.02,.05,lo,hi),"position_and_deceleration_conflict_rejected");
  c=configured();c.private_node_handle_.data.erase("joint_position_upper_limits");check(!c.configure(),"unpaired_position");
  c=configured();c.private_node_handle_.data["joint_position_upper_limits"].pop_back();check(!c.configure(),"position_dimension");
  c=configured();c.private_node_handle_.data["joint_position_lower_limits"][0]=std::numeric_limits<double>::quiet_NaN();check(!c.configure(),"position_nan");
  c=configured();c.private_node_handle_.data["joint_position_lower_limits"][0]=11.;c.private_node_handle_.data["joint_position_upper_limits"][0]=12.;check(!c.configure(),"empty_intersection");
  c=configured();c.private_node_handle_.data["joint_velocity_limits"].pop_back();check(!c.configure(),"velocity_dimension");
  c=configured();c.private_node_handle_.data["joint_velocity_limits"][0]=std::numeric_limits<double>::quiet_NaN();check(!c.configure(),"nan_not_hidden_by_min");
  c=configured();c.private_node_handle_.data["joint_acceleration_limits"].pop_back();check(!c.configure(),"acceleration_dimension");
  c=configured();c.private_node_handle_.data["joint_acceleration_limits"][0]=0.;check(!c.configure(),"acceleration_zero");
  std::cout<<"通过 "<<checks<<" 项实际C++限值断言（ROS参数使用桩，未运行ROS/仿真/真机）\n";
}
'''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--eigen', type=Path, required=True)
    parser.add_argument('--zig', type=str)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    yaml = (root/'config/elfin5_qp_controller.yaml').read_text(encoding='utf-8')
    for key, degrees in [('joint_position_upper_limits',[360,135,153,360,180,360]),
                         ('joint_velocity_limits',[180,180,180,180,200,200])]:
        actual = value(yaml,key)
        assert len(actual) == 6
        assert all(abs(a-math.radians(d)*.9)<1e-12 for a,d in zip(actual,degrees)), key
    assert value(yaml,'joint_position_lower_limits') == [-a for a in value(yaml,'joint_position_upper_limits')]
    assert value(yaml,'joint_acceleration_limits') == [1.]*6
    assert value(yaml,'position_limit_margin') == 0.
    assert value(yaml,'velocity_limit_scale') == value(yaml,'acceleration_limit_scale') == 1.
    site = (root/'config/elfin5_real_robot.yaml').read_text(encoding='utf-8')
    assert value(site,'acceleration_limit_scale') == .25
    assert value(site,'velocity_limit_scale') == .05
    source = (root/'src_gazebo/qp_cartesian_controller.cpp').read_text(encoding='utf-8')
    first = source.index('  std::vector<double> configured_position_lower_limits;')
    last = source.index('  robot_state_.reset', first)
    bfirst = source.index('bool QPCartesianController::buildVelocityBounds(')
    blast = source.index('Eigen::Isometry3d QPCartesianController::poseToIsometry', bfirst)
    parameters = '\n'.join('c.private_node_handle_.data["'+key+'"]='+str(value(yaml,key)).replace('[','{').replace(']','}')+';'
                            for key in ['joint_position_lower_limits','joint_position_upper_limits',
                                        'joint_velocity_limits','joint_acceleration_limits'])
    program = HARNESS.replace('// ACTUAL_CONFIGURATION',source[first:last]).replace(
        '// ACTUAL_VELOCITY_BOUNDS',source[bfirst:blast]).replace('// PARAMETERS',parameters)
    with tempfile.TemporaryDirectory(prefix='elfin5_limits_') as folder:
        path = Path(folder)
        (path/'limits.cpp').write_text(program,encoding='utf-8')
        exe = path/('limits.exe' if args.zig else 'limits')
        # 旧Eigen3.3.4与新Clang的模板解析兼容；Ubuntu/GCC不需要该参数。
        compiler = [args.zig,'c++','-fdelayed-template-parsing'] if args.zig else ['g++']
        subprocess.check_call(compiler+['-std=c++14','-O2','-Wall','-Wextra','-Wpedantic',
                              '-isystem',str(args.eigen.resolve()),str(path/'limits.cpp'),'-o',str(exe)])
        subprocess.check_call([str(exe)])
    print('六轴手册90%换算、默认加速度与低速候选参数检查通过')


if __name__ == '__main__':
    main()
