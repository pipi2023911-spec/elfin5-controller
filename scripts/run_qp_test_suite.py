#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Elfin5 QP 控制器的九轮 Gazebo 集成测试运行器。

兼容 ROS Melodic/Python 2。脚本通过 MoveIt 设置初始关节姿态，通过控制器
已有 ROS 接口发布绝对目标并记录指标；它不会修改控制器参数或机器人模型。
"""

from __future__ import print_function

import argparse
import codecs
import csv
import datetime
import json
import math
import os
import signal
import subprocess
import sys
import threading
import time

import moveit_commander
import rospkg
import rospy
import yaml
from controller_manager_msgs.srv import ListControllers
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PointStamped, PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from std_srvs.srv import SetBool
from trajectory_msgs.msg import JointTrajectory


FATAL_STATES = set(["QP_INFEASIBLE", "SINGULARITY_STOP",
                    "INVALID_INPUT", "STALE_INPUT"])

try:
    TEXT_TYPE = unicode
except NameError:
    TEXT_TYPE = str


def as_text(value):
    # 兼容 Python 2 的 ROS 字节字符串及 Python 3 的文本字符串。
    if isinstance(value, bytes):
        return value.decode("utf-8", "replace")
    return TEXT_TYPE(value)


def finite(value):
    return value is not None and not math.isnan(value) and not math.isinf(value)


def compact_states(states):
    result = []
    for state in states:
        if not result or result[-1] != state:
            result.append(state)
    return result


class InputPublisher(object):
    def __init__(self, config):
        topics = config["topics"]
        self.frame = config["absolute_frame"]
        self.base_cfg = config["base_pose"]
        self.base_pub = rospy.Publisher(topics["base_pose"], PoseStamped,
                                        queue_size=1)
        self.target_pub = rospy.Publisher(topics["target_point"], PointStamped,
                                          queue_size=1)
        self.base_enabled = True
        self.target_enabled = False
        self.target = None
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self._loop)
        self.thread.daemon = True
        self.thread.start()

    def set_target(self, target):
        with self.lock:
            self.target = list(target)
            self.target_enabled = True

    def stop_target(self):
        with self.lock:
            self.target_enabled = False

    def set_base_enabled(self, enabled):
        with self.lock:
            self.base_enabled = bool(enabled)

    def shutdown(self):
        self.stop_event.set()
        self.thread.join(2.0)

    def _loop(self):
        rate = max(float(self.base_cfg.get("publish_rate", 10.0)), 1.0)
        delay = 1.0 / rate
        while not self.stop_event.is_set() and not rospy.is_shutdown():
            with self.lock:
                base_enabled = self.base_enabled
                target_enabled = self.target_enabled
                target = list(self.target) if self.target is not None else None
            now = rospy.Time.now()
            if base_enabled:
                message = PoseStamped()
                message.header.stamp = now
                message.header.frame_id = self.frame
                p = self.base_cfg["position"]
                q = self.base_cfg["orientation"]
                message.pose.position.x = p[0]
                message.pose.position.y = p[1]
                message.pose.position.z = p[2]
                message.pose.orientation.x = q[0]
                message.pose.orientation.y = q[1]
                message.pose.orientation.z = q[2]
                message.pose.orientation.w = q[3]
                self.base_pub.publish(message)
            if target_enabled and target is not None:
                message = PointStamped()
                message.header.stamp = now
                message.header.frame_id = self.frame
                message.point.x, message.point.y, message.point.z = target
                self.target_pub.publish(message)
            time.sleep(delay)


class Monitor(object):
    def __init__(self, config):
        self.config = config
        self.lock = threading.Lock()
        self.active_round = None
        self.latest_joint = None
        self.latest_point = None
        self.latest_error = None
        self.latest_sigma = None
        self.latest_state = None
        self.latest_detail = ""
        self.latest_level = None
        self.command_count = 0
        self.states = []
        self.errors = []
        self.sigmas = []
        self.joints = []
        self.commands = []
        self.targets = []
        topics = config["topics"]
        rospy.Subscriber(topics["joint_states"], JointState,
                         self._joint_callback, queue_size=20)
        rospy.Subscriber(topics["current_point"], PointStamped,
                         self._point_callback, queue_size=20)
        rospy.Subscriber(topics["position_error"], Float64,
                         self._error_callback, queue_size=50)
        rospy.Subscriber(topics["singular_value"], Float64,
                         self._sigma_callback, queue_size=50)
        rospy.Subscriber(topics["diagnostics"], DiagnosticArray,
                         self._diagnostics_callback, queue_size=50)
        rospy.Subscriber(topics["command"], JointTrajectory,
                         self._command_callback, queue_size=100)

    def begin_round(self, name):
        with self.lock:
            self.active_round = name

    def end_round(self):
        with self.lock:
            self.active_round = None

    def add_target(self, target):
        with self.lock:
            self.targets.append({"round": self.active_round,
                                 "wall_time": time.time(),
                                 "x": target[0], "y": target[1],
                                 "z": target[2]})

    def snapshot(self):
        with self.lock:
            return {
                "joint": self.latest_joint,
                "point": self.latest_point,
                "error": self.latest_error,
                "sigma": self.latest_sigma,
                "state": self.latest_state,
                "detail": self.latest_detail,
                "level": self.latest_level,
                "command_count": self.command_count,
            }

    def round_rows(self, container, round_name):
        with self.lock:
            return [dict(row) for row in container if row["round"] == round_name]

    def _base_row(self):
        return {"round": self.active_round, "wall_time": time.time(),
                "ros_time": rospy.Time.now().to_sec()}

    def _joint_callback(self, message):
        positions = dict(zip(message.name, message.position))
        velocities = dict(zip(message.name, message.velocity))
        with self.lock:
            self.latest_joint = (positions, velocities)
            if self.active_round:
                row = self._base_row()
                for index, name in enumerate(self.config["joint_names"]):
                    row["q%d" % (index + 1)] = positions.get(name, "")
                    row["qd%d" % (index + 1)] = velocities.get(name, "")
                self.joints.append(row)

    def _point_callback(self, message):
        with self.lock:
            self.latest_point = [message.point.x, message.point.y, message.point.z]

    def _error_callback(self, message):
        with self.lock:
            self.latest_error = message.data
            if self.active_round:
                row = self._base_row()
                row["position_error"] = message.data
                self.errors.append(row)

    def _sigma_callback(self, message):
        with self.lock:
            self.latest_sigma = message.data
            if self.active_round:
                row = self._base_row()
                row["minimum_singular_value"] = message.data
                self.sigmas.append(row)

    def _diagnostics_callback(self, message):
        for status in message.status:
            if status.name != "elfin5_qp_controller":
                continue
            values = dict((item.key, item.value) for item in status.values)
            state = as_text(status.message).split(":", 1)[0]
            detail = as_text(status.message).split(":", 1)[1].strip() \
                if ":" in as_text(status.message) else as_text(status.message)
            with self.lock:
                self.latest_state = state
                self.latest_detail = detail
                self.latest_level = status.level
                if self.active_round:
                    row = self._base_row()
                    row.update({
                        "level": status.level,
                        "state": state,
                        "detail": detail,
                        "position_error": values.get("position_error_m", ""),
                        "minimum_singular_value": values.get(
                            "minimum_singular_value", ""),
                        "qp_iterations": values.get("qp_iterations", ""),
                        "qp_projected_gradient_norm": values.get(
                            "qp_projected_gradient_norm", ""),
                    })
                    self.states.append(row)

    def _command_callback(self, message):
        with self.lock:
            self.command_count += 1
            if not self.active_round:
                return
            row = self._base_row()
            row["header_stamp"] = message.header.stamp.to_sec()
            if message.points:
                point = message.points[-1]
                row["time_from_start"] = point.time_from_start.to_sec()
                for index in range(6):
                    row["position_%d" % (index + 1)] = \
                        point.positions[index] if index < len(point.positions) else ""
                    row["velocity_%d" % (index + 1)] = \
                        point.velocities[index] if index < len(point.velocities) else ""
            self.commands.append(row)


class TestRunner(object):
    def __init__(self, config, args, output_dir):
        self.config = config
        self.args = args
        self.output_dir = output_dir
        self.monitor = Monitor(config)
        self.inputs = InputPublisher(config)
        self.enable_service = None
        self.list_controllers = None
        self.group = None
        self.bag_process = None
        self.results = []

    def initialize(self):
        topics = self.config["topics"]
        timeout = self.config["timing"]["startup_timeout"]
        rospy.loginfo(u"正在等待控制器服务及反馈话题")
        rospy.wait_for_service(topics["enable_service"], timeout=timeout)
        rospy.wait_for_service(topics["controller_list_service"], timeout=timeout)
        self.enable_service = rospy.ServiceProxy(topics["enable_service"], SetBool)
        self.list_controllers = rospy.ServiceProxy(
            topics["controller_list_service"], ListControllers)
        rospy.wait_for_message(topics["joint_states"], JointState, timeout=timeout)
        rospy.wait_for_message(topics["current_point"], PointStamped,
                               timeout=timeout)
        self._check_controllers()
        moveit_commander.roscpp_initialize(sys.argv)
        self.group = moveit_commander.MoveGroupCommander(
            self.config["planning_group"])
        moveit_cfg = self.config["moveit"]
        self.group.set_max_velocity_scaling_factor(
            moveit_cfg["velocity_scaling"])
        self.group.set_max_acceleration_scaling_factor(
            moveit_cfg["acceleration_scaling"])
        self._write_parameters()

    def shutdown(self):
        try:
            if self.enable_service:
                self.enable_service(False)
        except Exception:
            pass
        self.inputs.stop_target()
        self.inputs.set_base_enabled(True)
        self.inputs.shutdown()
        self._stop_bag()
        try:
            moveit_commander.roscpp_shutdown()
        except Exception:
            pass

    def start_bag(self):
        if self.args.no_bag:
            return
        topics = self.config["topics"]
        bag_path = os.path.join(self.output_dir, "qp_test.bag")
        command = ["rosbag", "record", "-O", bag_path,
                   topics["joint_states"], topics["command"],
                   topics["base_pose"], topics["target_point"],
                   topics["current_point"], topics["position_error"],
                   topics["singular_value"], topics["diagnostics"]]
        self.bag_process = subprocess.Popen(command, preexec_fn=os.setsid)
        time.sleep(1.0)

    def run_round(self, round_name, round_cfg):
        result = {"round": round_name, "title": round_cfg["title"],
                  "status": "FAIL", "reason": u"尚未运行",
                  "started_at": datetime.datetime.now().isoformat()}
        self.monitor.begin_round(round_name)
        try:
            if round_cfg.get("requires_deep_singularity_permission", False) and \
                    not self.args.allow_deep_singularity:
                result["status"] = "SKIPPED"
                result["reason"] = u"需要显式指定 --allow-deep-singularity"
                return result
            self._prepare_initial_pose(round_name, round_cfg, result)
            if self.args.dry_run:
                result["status"] = "DRY_RUN"
                result["reason"] = u"环境和初始状态检查通过"
                return result
            kind = round_cfg["kind"]
            if kind == "retarget":
                self._run_retarget(round_cfg, result)
            elif kind == "stale_input":
                self._run_stale_input(round_cfg, result)
            else:
                self._run_targets(round_cfg, result)
            self._evaluate_metrics(round_name, round_cfg, result)
            return result
        except Exception as error:
            result["status"] = "FAIL"
            result["reason"] = as_text(error)
            return result
        finally:
            result["finished_at"] = datetime.datetime.now().isoformat()
            try:
                self.enable_service(False)
            except Exception:
                pass
            self.inputs.stop_target()
            self.inputs.set_base_enabled(True)
            self.monitor.end_round()

    def _prepare_initial_pose(self, round_name, round_cfg, result):
        self.inputs.set_base_enabled(True)
        self.inputs.stop_target()
        time.sleep(0.5)
        response = self.enable_service(False)
        if not response.success:
            raise RuntimeError(u"关闭服务失败：" + as_text(response.message))
        self._wait_state("IDLE", self.config["timing"]["state_timeout"])
        time.sleep(0.5)
        self._assert_command_quiet()
        if self.args.dry_run:
            result["initial_joints_requested"] = round_cfg["initial_joints"]
            return
        rospy.loginfo(u"%s：正在移动到初始关节位置 %s",
                      round_name, round_cfg["initial_joints"])
        self.group.set_start_state_to_current_state()
        success = self.group.go(round_cfg["initial_joints"], wait=True)
        self.group.stop()
        self.group.clear_pose_targets()
        if not success:
            raise RuntimeError(u"MoveIt 未能执行初始关节姿态")
        time.sleep(self.config["timing"]["moveit_settle_time"])
        self._check_joint_goal(round_cfg["initial_joints"])
        sigma = self._wait_numeric("sigma")
        sigma_range = round_cfg["expected_initial_sigma"]
        result["initial_sigma"] = sigma
        result["initial_joints_requested"] = round_cfg["initial_joints"]
        result["initial_joints_actual"] = self._current_joint_vector()
        if sigma < sigma_range[0] or sigma > sigma_range[1]:
            raise RuntimeError(
                u"初始最小奇异值 %.6g 超出预期范围 [%.6g, %.6g]" %
                (sigma, sigma_range[0], sigma_range[1]))

    def _run_targets(self, round_cfg, result):
        relative_to_current = round_cfg.get("relative_to", "initial") == "current"
        anchor = list(self._wait_point())
        self._start_enabled_target(anchor, round_cfg["target_offsets"][0])
        targets = []
        for index, offset in enumerate(round_cfg["target_offsets"]):
            if index > 0:
                if relative_to_current:
                    anchor = list(self._wait_point())
                target = [anchor[i] + offset[i] for i in range(3)]
                self.inputs.set_target(target)
                self.monitor.add_target(target)
                time.sleep(self.config["timing"]["target_settle_time"])
            else:
                target = [anchor[i] + offset[i] for i in range(3)]
            targets.append(target)
            state = self._wait_for_target_outcome(
                self.config["timing"]["reach_timeout"],
                round_cfg.get("allowed_safe_terminal_states", []))
            if state != "REACHED":
                if state in round_cfg.get("allowed_safe_terminal_states", []):
                    result["status"] = "PASS_SAFE_HOLD"
                    result["reason"] = u"进入允许的安全终止状态：" + state
                    result["terminal_state"] = state
                    result["targets"] = targets
                    return
                raise RuntimeError(u"目标执行结束时的状态：" + str(state))
        result["targets"] = targets
        result["terminal_state"] = "REACHED"
        result["status"] = "PASS"
        result["reason"] = u"全部目标已到达"

    def _run_retarget(self, round_cfg, result):
        anchor = list(self._wait_point())
        offsets = round_cfg["target_offsets"]
        first = [anchor[i] + offsets[0][i] for i in range(3)]
        second = [anchor[i] + offsets[1][i] for i in range(3)]
        self.inputs.set_target(first)
        self.monitor.add_target(first)
        time.sleep(self.config["timing"]["target_settle_time"])
        response = self.enable_service(True)
        if not response.success:
            raise RuntimeError(u"启用请求被拒绝：" + as_text(response.message))
        end = time.time() + round_cfg["switch_after"]
        while time.time() < end:
            state = self.monitor.snapshot()["state"]
            if state in FATAL_STATES:
                raise RuntimeError(u"切换目标前出现故障状态：" + state)
            time.sleep(0.05)
        self.inputs.set_target(second)
        self.monitor.add_target(second)
        state = self._wait_for_target_outcome(
            self.config["timing"]["reach_timeout"], [])
        if state != "REACHED":
            raise RuntimeError(u"在线切换目标后的终止状态：" + str(state))
        result["targets"] = [first, second]
        result["terminal_state"] = state
        result["status"] = "PASS"
        result["reason"] = u"在线切换后的目标已到达"

    def _run_stale_input(self, round_cfg, result):
        anchor = list(self._wait_point())
        offset = round_cfg["target_offsets"][0]
        target = [anchor[i] + offset[i] for i in range(3)]
        self.inputs.set_target(target)
        self.monitor.add_target(target)
        time.sleep(self.config["timing"]["target_settle_time"])
        response = self.enable_service(True)
        if not response.success:
            raise RuntimeError(u"启用请求被拒绝：" + as_text(response.message))
        self._wait_any_state(["RUNNING", "SINGULARITY_RECOVERY"], 5.0)
        self.inputs.stop_target()
        self._wait_state("STALE_INPUT",
                         self.config["limits"]["input_timeout"] + 2.0)
        result["target_timeout_passed"] = True
        self.enable_service(False)
        self.inputs.set_target(target)
        self.inputs.set_base_enabled(True)
        time.sleep(0.5)
        self._wait_state("IDLE", 5.0)
        response = self.enable_service(True)
        if not response.success:
            raise RuntimeError(u"第二次启用请求被拒绝：" + as_text(response.message))
        self._wait_any_state(["RUNNING", "SINGULARITY_RECOVERY"], 5.0)
        self.inputs.set_base_enabled(False)
        self._wait_state("STALE_INPUT",
                         self.config["limits"]["input_timeout"] + 2.0)
        result["base_timeout_passed"] = True
        result["terminal_state"] = "STALE_INPUT"
        result["status"] = "PASS"
        result["reason"] = u"目标及基座输入超时保护检查通过"

    def _start_enabled_target(self, anchor, offset):
        target = [anchor[i] + offset[i] for i in range(3)]
        self.inputs.set_target(target)
        self.monitor.add_target(target)
        time.sleep(self.config["timing"]["target_settle_time"])
        response = self.enable_service(True)
        if not response.success:
            raise RuntimeError(u"启用请求被拒绝：" + as_text(response.message))

    def _wait_for_target_outcome(self, timeout, allowed_safe_states):
        deadline = time.time() + timeout
        while time.time() < deadline and not rospy.is_shutdown():
            state = self.monitor.snapshot()["state"]
            if state == "REACHED" or state in allowed_safe_states:
                return state
            if state in FATAL_STATES:
                return state
            time.sleep(self.config["timing"]["sample_period"])
        return "TIMEOUT"

    def _evaluate_metrics(self, round_name, round_cfg, result):
        errors = self.monitor.round_rows(self.monitor.errors, round_name)
        sigmas = self.monitor.round_rows(self.monitor.sigmas, round_name)
        states = self.monitor.round_rows(self.monitor.states, round_name)
        commands = self.monitor.round_rows(self.monitor.commands, round_name)
        error_values = [row["position_error"] for row in errors
                        if finite(row["position_error"])]
        sigma_values = [row["minimum_singular_value"] for row in sigmas
                        if finite(row["minimum_singular_value"])]
        state_sequence = compact_states([row["state"] for row in states])
        result["state_sequence"] = state_sequence
        if error_values:
            result["initial_error"] = error_values[0]
            result["final_error"] = error_values[-1]
            result["minimum_error"] = min(error_values)
        if sigma_values:
            result["minimum_sigma"] = min(sigma_values)
            result["final_sigma"] = sigma_values[-1]
            if not all(finite(value) for value in sigma_values):
                result["status"] = "FAIL"
                result["reason"] = u"奇异值不是有限数值"
        max_velocity, max_acceleration = self._command_metrics(commands)
        result["maximum_command_velocity"] = max_velocity
        result["maximum_command_acceleration"] = max_acceleration
        if round_cfg["kind"] == "constraints" and result["status"] == "PASS":
            limits = self.config["limits"]
            tolerance = limits["command_limit_tolerance"]
            if max_velocity > limits["maximum_joint_velocity"] + tolerance:
                result["status"] = "FAIL"
                result["reason"] = u"命令速度超过限值"
            if max_acceleration > limits["maximum_joint_acceleration"] * 1.25:
                result["status"] = "FAIL"
                result["reason"] = u"命令加速度超过限值"
        if round_cfg["kind"] in ["singular_recovery", "singular_protection",
                                 "deep_singularity"] and sigma_values:
            improvement = max(sigma_values) - sigma_values[0]
            result["maximum_sigma_improvement"] = improvement
            required = round_cfg.get("minimum_sigma_improvement", 0.0)
            safe_terminal = result.get("terminal_state") in \
                round_cfg.get("allowed_safe_terminal_states", [])
            if improvement < required and not safe_terminal:
                result["status"] = "FAIL"
                result["reason"] = u"最小奇异值的改善幅度未达到 %.6g" % required

    def _command_metrics(self, commands):
        maximum_velocity = 0.0
        maximum_acceleration = 0.0
        previous = None
        for row in commands:
            velocities = []
            for index in range(6):
                value = row.get("velocity_%d" % (index + 1), "")
                if value == "":
                    velocities = []
                    break
                velocities.append(float(value))
                maximum_velocity = max(maximum_velocity, abs(float(value)))
            if velocities and previous is not None:
                dt = row["ros_time"] - previous[0]
                if dt > 1e-4:
                    for current, old in zip(velocities, previous[1]):
                        maximum_acceleration = max(
                            maximum_acceleration, abs(current - old) / dt)
            if velocities:
                previous = (row["ros_time"], velocities)
        return maximum_velocity, maximum_acceleration

    def _wait_state(self, expected, timeout):
        return self._wait_any_state([expected], timeout)

    def _wait_any_state(self, expected, timeout):
        deadline = time.time() + timeout
        while time.time() < deadline and not rospy.is_shutdown():
            state = self.monitor.snapshot()["state"]
            if state in expected:
                return state
            time.sleep(0.05)
        raise RuntimeError(u"等待控制器状态 %s 超时；当前状态为 %s" %
                           (expected, self.monitor.snapshot()["state"]))

    def _wait_numeric(self, key):
        deadline = time.time() + self.config["timing"]["state_timeout"]
        while time.time() < deadline:
            value = self.monitor.snapshot()[key]
            if finite(value):
                return value
            time.sleep(0.05)
        raise RuntimeError(u"等待有限数值超时：" + key)

    def _wait_point(self):
        deadline = time.time() + self.config["timing"]["state_timeout"]
        while time.time() < deadline:
            point = self.monitor.snapshot()["point"]
            if point is not None and all(finite(value) for value in point):
                return point
            time.sleep(0.05)
        raise RuntimeError(u"等待当前末端位置超时")

    def _current_joint_vector(self):
        joint = self.monitor.snapshot()["joint"]
        if joint is None:
            raise RuntimeError(u"关节反馈不可用")
        positions = joint[0]
        missing = [name for name in self.config["joint_names"]
                   if name not in positions]
        if missing:
            raise RuntimeError(u"关节反馈缺少：" + ",".join(missing))
        return [positions[name] for name in self.config["joint_names"]]

    def _check_joint_goal(self, requested):
        actual = self._current_joint_vector()
        tolerance = self.config["limits"]["joint_goal_tolerance"]
        maximum_error = max(abs(a - b) for a, b in zip(actual, requested))
        if maximum_error > tolerance:
            raise RuntimeError(u"MoveIt 关节目标误差 %.6g 超过容差 %.6g" %
                               (maximum_error, tolerance))

    def _assert_command_quiet(self):
        quiet_time = self.config["timing"]["command_quiet_time"]
        before = self.monitor.snapshot()["command_count"]
        time.sleep(quiet_time)
        after = self.monitor.snapshot()["command_count"]
        if after - before > 1:
            raise RuntimeError(
                u"QP 关闭时命令话题仍以 %.2f Hz 发布；"
                u"基座位姿或状态检查尚未就绪" %
                ((after - before) / quiet_time))

    def _check_controllers(self):
        response = self.list_controllers()
        states = dict((item.name, item.state) for item in response.controller)
        if states.get("joint_state_controller") != "running":
            raise RuntimeError(u"joint_state_controller 未处于运行状态")
        if states.get("elfin_arm_controller") != "running":
            raise RuntimeError(u"elfin_arm_controller 未处于运行状态")

    def _write_parameters(self):
        path = os.path.join(self.output_dir, "parameters.yaml")
        data = {
            "test_config": self.config,
            "controller_parameters": rospy.get_param(
                "/elfin5_qp_controller", {}),
            "use_sim_time": rospy.get_param("/use_sim_time", False),
        }
        content = yaml.safe_dump(data, default_flow_style=False,
                                 allow_unicode=True)
        if not isinstance(content, TEXT_TYPE):
            content = content.decode("utf-8")
        with codecs.open(path, "w", "utf-8") as stream:
            stream.write(content)

    def _stop_bag(self):
        if self.bag_process is None or self.bag_process.poll() is not None:
            return
        os.killpg(os.getpgid(self.bag_process.pid), signal.SIGINT)
        deadline = time.time() + 10.0
        while self.bag_process.poll() is None and time.time() < deadline:
            time.sleep(0.1)
        if self.bag_process.poll() is None:
            self.bag_process.terminate()


def write_csv(path, rows, fieldnames):
    # Python 2.7 的 csv 模块要求二进制文件并且不能直接处理 unicode 值。
    with open(path, "wb") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames,
                                extrasaction="ignore")
        writer.writeheader()
        for row in rows:
            encoded = {}
            for key, value in row.items():
                if isinstance(value, TEXT_TYPE) and not isinstance(value, str):
                    encoded[key] = value.encode("utf-8")
                else:
                    encoded[key] = value
            writer.writerow(encoded)


def write_outputs(output_dir, runner):
    monitor = runner.monitor
    joint_fields = ["round", "wall_time", "ros_time"] + \
        ["q%d" % i for i in range(1, 7)] + \
        ["qd%d" % i for i in range(1, 7)]
    command_fields = ["round", "wall_time", "ros_time", "header_stamp",
                      "time_from_start"] + \
        ["position_%d" % i for i in range(1, 7)] + \
        ["velocity_%d" % i for i in range(1, 7)]
    state_fields = ["round", "wall_time", "ros_time", "level", "state",
                    "detail", "position_error", "minimum_singular_value",
                    "qp_iterations", "qp_projected_gradient_norm"]
    write_csv(os.path.join(output_dir, "joint_states.csv"),
              monitor.joints, joint_fields)
    write_csv(os.path.join(output_dir, "trajectory_commands.csv"),
              monitor.commands, command_fields)
    write_csv(os.path.join(output_dir, "controller_state.csv"),
              monitor.states, state_fields)
    write_csv(os.path.join(output_dir, "position_error.csv"), monitor.errors,
              ["round", "wall_time", "ros_time", "position_error"])
    write_csv(os.path.join(output_dir, "singular_value.csv"), monitor.sigmas,
              ["round", "wall_time", "ros_time", "minimum_singular_value"])
    write_csv(os.path.join(output_dir, "targets.csv"), monitor.targets,
              ["round", "wall_time", "x", "y", "z"])
    json_content = json.dumps(runner.results, indent=2, sort_keys=True,
                              ensure_ascii=False)
    if not isinstance(json_content, TEXT_TYPE):
        json_content = json_content.decode("utf-8")
    with codecs.open(os.path.join(output_dir, "rounds.json"),
                     "w", "utf-8") as stream:
        stream.write(json_content)
    summary = [u"# Elfin5 QP 自动化测试结果", u"",
               u"生成时间：`%s`" % datetime.datetime.now().isoformat(), u"",
               u"| 轮次 | 名称 | 结果 | 终态 | 初始σ | 最小σ | 最终误差 | 原因 |",
               u"|---|---|---|---|---:|---:|---:|---|"]
    for item in runner.results:
        summary.append(u"| {round} | {title} | {status} | {terminal} | "
                       u"{initial_sigma} | {minimum_sigma} | {final_error} | "
                       u"{reason} |".format(
                           round=item.get("round", ""),
                           title=item.get("title", ""),
                           status=item.get("status", ""),
                           terminal=item.get("terminal_state", ""),
                           initial_sigma=item.get("initial_sigma", ""),
                           minimum_sigma=item.get("minimum_sigma", ""),
                           final_error=item.get("final_error", ""),
                           reason=as_text(item.get("reason", u""))
                           .replace(u"|", u"/")))
    summary.extend([u"", u"## 逐轮详情", u""])
    for item in runner.results:
        detail = json.dumps(item, indent=2, sort_keys=True, ensure_ascii=False)
        if not isinstance(detail, TEXT_TYPE):
            detail = detail.decode("utf-8")
        summary.extend([u"### %s — %s" % (item["round"], item["title"]), u"",
                        u"```json", detail, u"```", u""])
    with codecs.open(os.path.join(output_dir, "summary.md"),
                     "w", "utf-8") as stream:
        stream.write(u"\n".join(summary))


def load_config(path):
    with codecs.open(path, "r", "utf-8") as stream:
        return yaml.safe_load(stream)


def resolve_output(args):
    if args.output:
        output = os.path.abspath(os.path.expanduser(args.output))
    else:
        package = rospkg.RosPack().get_path("elfin5_qp_controller")
        stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        output = os.path.join(package, "docs", "test_runs", stamp)
    try:
        if not os.path.isdir(output):
            os.makedirs(output)
    except OSError:
        stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        output = os.path.expanduser(
            os.path.join("~", ".ros", "elfin5_qp_controller", "test_runs", stamp))
        if not os.path.isdir(output):
            os.makedirs(output)
    return output


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default=None,
                        help=u"测试 YAML 路径，默认使用包内配置")
    parser.add_argument("--rounds", default="R1,R2,R3,R4,R5,R6,R7,R8,R9")
    parser.add_argument("--output", default=None)
    parser.add_argument("--non-interactive", action="store_true")
    parser.add_argument("--continue-on-failure", action="store_true")
    parser.add_argument("--allow-deep-singularity", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--no-bag", action="store_true")
    return parser.parse_args()


def main():
    args = parse_args()
    package = rospkg.RosPack().get_path("elfin5_qp_controller")
    config_path = args.config or os.path.join(
        package, "config", "qp_test_suite.yaml")
    config = load_config(config_path)
    requested = [item.strip() for item in args.rounds.split(",") if item.strip()]
    unknown = [item for item in requested if item not in config["rounds"]]
    if unknown:
        raise RuntimeError(u"未知测试轮次：" + ",".join(unknown))
    output_dir = resolve_output(args)
    rospy.init_node("elfin5_qp_test_suite", anonymous=False)
    runner = TestRunner(config, args, output_dir)
    print(u"结果目录：" + output_dir)
    try:
        runner.initialize()
        runner.start_bag()
        for round_name in requested:
            round_cfg = config["rounds"][round_name]
            if not args.non_interactive and not args.dry_run:
                prompt = (u"输入 RUN %s 执行“%s”，输入其他内容则跳过：" %
                          (round_name, round_cfg["title"]))
                # Python 2 raw_input() tries to coerce a unicode prompt through
                # ASCII.  Round titles come from YAML as unicode, so encode the
                # prompt explicitly for an UTF-8 ROS terminal.
                if sys.version_info[0] < 3:
                    answer = raw_input(prompt.encode("utf-8"))
                else:
                    answer = input(prompt)
                if answer.strip() != "RUN " + round_name:
                    runner.results.append({"round": round_name,
                                           "title": round_cfg["title"],
                                           "status": "SKIPPED",
                                           "reason": u"操作人员已跳过"})
                    write_outputs(output_dir, runner)
                    continue
            result = runner.run_round(round_name, round_cfg)
            runner.results.append(result)
            write_outputs(output_dir, runner)
            print(u"%s：%s — %s" %
                  (round_name, result["status"], result["reason"]))
            if result["status"] == "FAIL" and not args.continue_on_failure:
                print(u"测试失败后已停止；如需继续须显式指定 --continue-on-failure")
                break
    finally:
        runner.shutdown()
        write_outputs(output_dir, runner)
        print(u"最终报告：" + os.path.join(output_dir, "summary.md"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
