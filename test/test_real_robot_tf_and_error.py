#!/usr/bin/env python
# -*- coding: utf-8 -*-
# 不依赖ROS运行环境的模型、误差和接线静态检查。
# 直接加载两个新增脚本的纯函数，不导入rosbag，也不启动任何机械臂节点。
# URDF使用最小结构夹具，验证保留 world 锚点和厂家原始内部固定旋转。
# 误差测试覆盖名称映射、符号、NaN、缺失段、角度分支及ACTIVE阶段覆盖率。
# launch检查只证明文本中的重映射存在；真实ROS图隔离另由rostest验证。
# 本文件的PASS不能解释为完整模型、运动学或真机验收通过。

from __future__ import print_function
import math
import os
import sys
import unittest
import xml.etree.ElementTree as ET

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
# 兼容Python2的imp与Python3的importlib加载器，让同一组纯函数检查可在开发机和Melodic执行。
def load_source(name, path):
    if sys.version_info[0] >= 3:
        import importlib.util
        spec = importlib.util.spec_from_file_location(name, path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module
    import imp
    return imp.load_source(name, path)

plot = load_source("real_error_plot", os.path.join(ROOT,"scripts","plot_real_robot_error.py"))
model = load_source("real_model", os.path.join(ROOT,"scripts","prepare_real_robot_description.py"))
# 最小URDF夹具，仅保留世界锚点和基座固定关系；不把它当成完整Elfin5模型。
XML = """<robot name="test"><link name="world"/><link name="elfin_base_link"/><link name="elfin_base"/>
<joint name="world_base" type="fixed"><parent link="world"/><child link="elfin_base_link"/><origin xyz="0 0 0" rpy="0 0 0"/></joint>
<joint name="elfin_base_joint" type="fixed"><parent link="elfin_base_link"/><child link="elfin_base"/><origin rpy="0 0 1.5707963267948966"/></joint></robot>"""

class OfflineChecks(unittest.TestCase):
    # 确认完整模型原样保留，包含 world 锚点及内部固定旋转。
    def test_base_rotation_preserved(self):
        root = ET.fromstring(model.prepare_urdf(XML))
        self.assertIn("world",[x.get("name") for x in root.findall("link")])
        self.assertEqual(model.prepare_urdf(XML), XML.encode("utf-8"))
        fixed=root.findall("joint")[1]
        self.assertEqual(fixed.get("name"),"elfin_base_joint")
        self.assertAlmostEqual(float(fixed.find("origin").get("rpy").split()[2]), math.pi/2)
    # 根与基座不重合时不能直接使用基座位姿广播根变换，必须拒绝。
    def test_nonidentity_anchor_rejected(self):
        with self.assertRaises(ValueError):model.prepare_urdf(XML.replace('xyz="0 0 0"','xyz="1 0 0"'))
    # 厂家内部基座关节的父链接不符时拒绝，不能改接到 world。
    def test_extra_world_parent_rejected(self):
        with self.assertRaises(ValueError):model.prepare_urdf(XML.replace('parent link="elfin_base_link"','parent link="world"'))
    # 供应商版本的内部固定旋转可以不同，检查工具不得将它硬编码为 90 度。
    def test_identity_internal_base_is_preserved(self):
        xml=XML.replace('1.5707963267948966','0')
        self.assertEqual(model.prepare_urdf(xml),xml.encode("utf-8"))
    # 世界根可以带其他分支或内容；保留模型后无需旧的“空根可删除”限制。
    def test_full_world_content_is_preserved(self):
        xml=XML.replace('<link name="world"/>','<link name="world"><visual/></link>')
        self.assertEqual(model.prepare_urdf(xml),xml.encode("utf-8"))
    # 重复父关节依然是非法 URDF，保留树结构检查。
    def test_multiple_parents_rejected(self):
        xml=XML.replace('</robot>','<joint name="extra" type="fixed"><parent link="world"/><child link="elfin_base"/></joint></robot>')
        with self.assertRaises(ValueError):model.prepare_urdf(xml)
    # 使用乱序名称和正负误差，检查重排与desired−actual的符号方向。
    def test_joint_name_order_and_sign(self):
        self.assertEqual(plot.joint_errors(["b","a"],[1.,2.],[1.5,1.],["a","b"]),[1.,-.5])
    # 异常输入必须抛错，不能误算成零误差或错轴误差。
    def test_joint_missing_nan_duplicate_rejected(self):
        for names,d,a in [(["a"],[0],[0]),(["a","a"],[0,0],[0,0]),(["a","b"],[float("nan"),0],[0,0])]:
            with self.assertRaises(ValueError):plot.joint_errors(names,d,a,["a","b"])
    # 3.1−(−3.1)必须保留6.2rad，不能隐式折成较小角差掩盖分支问题。
    def test_no_implicit_angle_wrap(self):
        self.assertAlmostEqual(plot.joint_errors(["a"],[3.1],[-3.1],["a"])[0],6.2)
    # 含NaN和大间隔的数据不产生有效时长；RMSE只使用真实有限样本。
    def test_missing_data_not_zero(self):
        s=plot.series_stats([0,.1,.2,.9],[1,float("nan"),-2,3],.3,1.5)
        self.assertAlmostEqual(s["sample_rmse"],math.sqrt(14./3))
        self.assertEqual(s["coverage"],0.)
    # 绘图序列须在缺失段插入NaN，使图线断开而不是跨越空白直连。
    def test_gap_breaks_line(self):
        x,y=plot.with_gaps([0,.1,1],[1,2,3],.3)
        self.assertEqual(len(x),4);self.assertTrue(math.isnan(y[2]))
    # ACTIVE过滤后的覆盖率分母只统计ACTIVE内部时间，停机时间不是丢帧。
    def test_coverage_excludes_nonactive_intervals(self):
        s=plot.series_stats([0,1,1.1,1.2,2],[float("nan"),1,2,3,float("nan")],.3,
                            eligible=[False,True,True,True,False])
        self.assertAlmostEqual(s["coverage"],1.)
        self.assertAlmostEqual(s["eligible_duration_seconds"],.2)
    # 确认V2源码写死的绝对输出被转向候选话题，不能仅靠节点namespace隔离。
    def test_launch_remaps_absolute_output(self):
        root=ET.parse(os.path.join(ROOT,"launch","elfin5_qp_real_robot.launch")).getroot()
        candidate=[n for n in root.findall(".//node") if n.get("name")=="qp_candidate"][0]
        remaps=dict((r.get("from"),r.get("to")) for r in candidate.findall("remap"))
        self.assertEqual(remaps["/elfin_arm_controller/command"],"/elfin5_real/candidate_trajectory")
        self.assertEqual(remaps["/joint_states"],"/joint_states")
        self.assertEqual(remaps["~base_pose"],"/base_pose")
        self.assertEqual(remaps["~target_point"],"/target_point")
        self.assertEqual(remaps["robot_description"],"/robot_description")
        self.assertFalse(root.findall(".//param[@name='robot_description']"))
        context=root.find(".//include/arg[@name='robot_description']")
        self.assertEqual(context.get("value"),"/robot_description")

    # 观测节点默认仍启动，但退出不应通过required关闭有效控制；控制节点仍必须存活。
    def test_display_is_not_a_required_control_node(self):
        root=ET.parse(os.path.join(ROOT,"launch","elfin5_qp_real_robot.launch")).getroot()
        nodes=dict((n.get("name"),n) for n in root.findall(".//node"))
        self.assertNotEqual(nodes["monitor"].get("required"),"true")
        self.assertNotIn("robot_state_publisher",nodes)
        for name in ("bridge","qp_candidate"):
            self.assertEqual(nodes[name].get("required"),"true")

if __name__=="__main__":
    unittest.main()
