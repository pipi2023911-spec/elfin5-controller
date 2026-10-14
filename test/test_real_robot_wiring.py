#!/usr/bin/env python
# -*- coding: utf-8 -*-
# ROS隔离接线测试夹具，只能由test_real_robot_wiring.test在独立测试master运行。
# 本文件故意生成假JointState、定位、目标、JTC状态和硬件OK，并提供假停止服务。
# 这些假接口用于验证网关状态机，严禁连接真机master或复制为现场监督适配器。
# 测试路径：READY仍零输出 → 显式启用后有输出 → 停发反馈触发锁存/stop →
# 恢复数据不自启 → reset后仍零输出 → 手动关闭回观察 → 停止失败锁存与显式重试。
# 该测试不验证EtherCAT通信、驱动器停车、模型精度或真实机械运动。

"""rostest ONLY: synthetic feedback on an isolated test master, never a real robot."""
import threading
import time
import unittest
import rospy
import rostest
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, PointStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from controller_manager_msgs.srv import ListControllers, ListControllersResponse
from controller_manager_msgs.msg import ControllerState, HardwareInterfaceResources
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse, SetBool

# 夹具生命周期由unittest管理；setUp连接假接口，tearDown退出后台发布线程并注销。
class Wiring(unittest.TestCase):
    # 初始化计数器与ROS收发端，随后启动20ms周期的合成数据流。
    def setUp(self):
        self.names=["elfin_joint%d" % i for i in range(1,7)]
        self.running=True;self.send_joint=True;self.outputs=[];self.stops=0;self.mode=""
        self.send_monitor=False;self.hardware_level=0;self.stop_success=True
        # 两个服务只在elfin5_test下使用；stop成功回执是假的，不表示物理停车能力。
        self.services=[
            rospy.Service("/elfin5_test/list",ListControllers,self.controllers),
            rospy.Service("/elfin5_test/stop",Trigger,self.stop)]
        self.subs=[
            rospy.Subscriber("/elfin5_test/command",JointTrajectory,self.output),
            rospy.Subscriber("/elfin5_real/bridge/state",String,self.state)]
        # 保持Publisher对象存活，确保ROS图能看到测试发布者，供网关检查命令来源。
        self.pubs={}
        for key,topic,typ in [
            ("j","/elfin5_test/joints",JointState),("b","/elfin5_test/base",PoseStamped),
            ("t","/elfin5_test/target",PointStamped),("c","/elfin5_test/controller",JointTrajectoryControllerState),
            ("h","/elfin5_test/hardware",DiagnosticArray),("q","/elfin5_real/qp_candidate/state",DiagnosticArray),
            ("m","/elfin5_real/monitor/tracking_state",DiagnosticArray),
            ("cmd","/elfin5_real/candidate_trajectory",JointTrajectory)]:
            self.pubs[key]=rospy.Publisher(topic,typ,queue_size=1)
        # 先等网关服务注册，再创建调用代理，避免启动时序导致偶然失败。
        rospy.wait_for_service("/elfin5_real/bridge/enable",10)
        self.enable=rospy.ServiceProxy("/elfin5_real/bridge/enable",SetBool)
        self.reset=rospy.ServiceProxy("/elfin5_real/bridge/reset",Trigger)
        self.thread=threading.Thread(target=self.feed);self.thread.daemon=True;self.thread.start()
    # 返回符合门禁契约的控制器名、running状态、位置接口和六轴占用资源。
    def controllers(self,req):
        c=ControllerState();c.name="elfin_arm_controller";c.state="running"
        c.type="position_controllers/JointTrajectoryController"
        r=HardwareInterfaceResources();r.hardware_interface="hardware_interface::PositionJointInterface";r.resources=self.names
        c.claimed_resources=[r]
        return ListControllersResponse(controller=[c])
    # 仅累计“网关确实发起stop请求”的次数；没有任何硬件动作。
    def stop(self,req):
        self.stops+=1;return TriggerResponse(success=self.stop_success,message="test_only_ack")
    # 只取状态名前缀，忽略随诊断原因变化的后半段文字。
    def state(self,msg):self.mode=msg.data.split(":")[0]
    # 收集实际输出通道上的消息，检查禁用/锁存时是否发生候选泄漏。
    def output(self,msg):self.outputs.append(msg)
    # 持续发送同一时间基准的假数据，让测试只通过send_joint开关制造反馈中断。
    def feed(self):
        while self.running and not rospy.is_shutdown():
            # 将时间稍置于当前之前，避免本机调度边界被误判成未来消息。
            stamp=rospy.Time.now()-rospy.Duration(.002)
            j=JointState();j.header.stamp=stamp;j.name=self.names;j.position=[0.]*6
            if self.send_joint:self.pubs["j"].publish(j)
            b=PoseStamped();b.header.stamp=stamp;b.header.frame_id="world";b.pose.orientation.w=1;self.pubs["b"].publish(b)
            t=PointStamped();t.header=b.header;self.pubs["t"].publish(t)
            c=JointTrajectoryControllerState();c.header.stamp=stamp;c.joint_names=self.names
            c.actual.positions=[0.]*6;c.desired.positions=[0.]*6;c.desired.velocities=[0.]*6;self.pubs["c"].publish(c)
            for key,name,identity,message in [("h","elfin5_hardware","test_robot","test_ok"),
                                                ("q","elfin5_qp_controller","test","REACHED: test_hold"),
                                                ("m","elfin5_real_tracking","test","valid")]:
                if key=="m" and not self.send_monitor:continue
                d=DiagnosticArray();d.header.stamp=stamp
                s=DiagnosticStatus();s.name=name;s.hardware_id=identity;s.message=message
                s.level=self.hardware_level if key=="h" else (1 if key=="m" else 0)
                d.status=[s];self.pubs[key].publish(d)
            # 合成零位保持候选；即使无位移，也应被OBSERVE/READY/FAULT状态完全隔离。
            cmd=JointTrajectory();cmd.header.stamp=stamp;cmd.joint_names=self.names
            p=JointTrajectoryPoint();p.positions=[0.]*6;p.velocities=[0.]*6;p.time_from_start=rospy.Duration(.1)
            cmd.points=[p];self.pubs["cmd"].publish(cmd)
            time.sleep(.02)
    # 使用有限墙钟等待，条件长期不满足会明确失败，不让测试无限挂起。
    def wait(self,predicate,seconds=5):
        end=time.time()+seconds
        while time.time()<end:
            if predicate():return
            time.sleep(.03)
        self.fail("Timed out; gate mode="+self.mode)
    # 按完整状态转换序列验证，重点防止“数据恢复后自动执行”和历史候选泄漏。
    def test_isolation_latch_timeout_and_explicit_restart(self):
        self.wait(lambda:self.mode=="READY")
        time.sleep(.15)
        self.assertEqual(len(self.outputs),0, "Candidate holds leaked before enable")
        result=self.enable(True);self.assertTrue(result.success,result.message)
        self.wait(lambda:len(self.outputs)>3)
        # 没有monitor也能就绪；显示WARN及随后停发不影响有效控制输入。
        self.send_monitor=True
        count=len(self.outputs);time.sleep(.4)
        self.assertEqual(self.mode,"ACTIVE");self.assertGreater(len(self.outputs),count)
        self.send_monitor=False
        time.sleep(.4);self.assertEqual(self.mode,"ACTIVE")
        # 仅中断真实关节反馈，其他输入继续更新，验证各输入的时效检查相互独立。
        self.send_joint=False
        self.wait(lambda:self.mode=="FAULT_LATCHED")
        self.wait(lambda:self.stops>0)
        count=len(self.outputs);time.sleep(.2)
        self.assertEqual(count,len(self.outputs))
        # 恢复反馈后仍必须保持锁存，不能直接返回ACTIVE。
        self.send_joint=True
        time.sleep(.4)
        self.assertEqual(self.mode,"FAULT_LATCHED")
        self.assertFalse(self.enable(True).success)
        # reset仅回READY/OBSERVE，接下来先检查仍无命令，再显式重新激活。
        self.assertTrue(self.reset().success)
        self.wait(lambda:self.mode=="READY")
        count=len(self.outputs);time.sleep(.1);self.assertEqual(count,len(self.outputs))
        self.assertTrue(self.enable(True).success)
        self.wait(lambda:len(self.outputs)>count)
        stops=self.stops
        self.assertTrue(self.enable(False).success)
        self.wait(lambda:self.stops>stops and self.mode=="READY")
        count=len(self.outputs);time.sleep(.2)
        self.assertEqual(count,len(self.outputs),"Normal close automatically resumed motion")
        # 真硬件故障仍锁存；停止失败不能通过reset清除，也不能自动重试。
        self.assertTrue(self.enable(True).success)
        self.wait(lambda:len(self.outputs)>count)
        self.stop_success=False;self.hardware_level=2
        self.wait(lambda:self.mode=="FAULT_LATCHED" and self.stops>stops+1)
        self.hardware_level=0
        time.sleep(.4)
        self.assertFalse(self.reset().success)
        self.assertFalse(self.enable(True).success)
        stops=self.stops;count=len(self.outputs);time.sleep(.2)
        self.assertEqual(self.stops,stops,"Stop was retried automatically")
        self.assertEqual(len(self.outputs),count)
        # 人工关闭服务仅重试停止；成功后依旧锁存，须reset再显式enable。
        self.stop_success=True
        self.assertTrue(self.enable(False).success)
        self.wait(lambda:self.stops>stops)
        time.sleep(.2)
        self.assertEqual(self.mode,"FAULT_LATCHED")
        self.assertTrue(self.reset().success)
        self.wait(lambda:self.mode=="READY")
        self.assertEqual(len(self.outputs),count)
    # 结束后台发布并注销接口，避免测试进程退出前残留消息影响后续用例。
    def tearDown(self):
        self.running=False;self.thread.join(2)
        for s in self.services:s.shutdown()
        for p in self.pubs.values():p.unregister()
        for s in self.subs:s.unregister()

# 由rostest管理节点和独立master；不得把此脚本rosrun到现场控制网络。
if __name__=="__main__":
    rospy.init_node("test_real_robot_wiring")
    rostest.rosrun("elfin5_qp_controller","real_robot_wiring",Wiring)
