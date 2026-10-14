#!/usr/bin/env python
# -*- coding: utf-8 -*-
# 【文件作用】生成移动基座用的 URDF，供真机 RViz 与 MoveIt 运动学共同使用。
# 【输入】--xacro 厂家模型路径，或 --urdf-file 已展开的 URDF（二选一）。
# 【输出】标准输出只写处理后的 UTF-8 XML，方便 roslaunch 用 command 加载参数。
# 【修改范围】只移除已核验的 world_base 固定关节及空 world 链接；保留内部关节/几何/旋转。
# 【失败处理】模型结构、根节点或连接关系不符时返回非零退出码，错误写到标准错误。
# 本脚本不编辑厂家文件、不发布 TF、不连接硬件；真实 world→base 动态变换由定位链提供。
# 兼容 Python 2.7 与 Python 3；文件头编码声明和 __future__ 导入的顺序不能破坏。

"""Generate a base-root URDF without editing the vendor model (Python 2.7/3)."""
from __future__ import print_function
import argparse
import subprocess
import sys
import xml.etree.ElementTree as ET

# 核心纯函数：接收 XML 字节或文本，返回新 XML 字节；独立于 ROS，便于离线测试。
def prepare_urdf(xml):
    root = ET.fromstring(xml)
    if root.tag != "robot":
        raise ValueError("Expected a robot URDF")
    # 按准确名称找到锚定世界的两个元素，数量必须唯一，避免误删其他模型结构。
    worlds = [x for x in root.findall("link") if x.get("name") == "world"]
    anchors = [x for x in root.findall("joint") if x.get("name") == "world_base"]
    if len(worlds) != 1 or len(anchors) != 1:
        raise ValueError("Expected exactly one world link and world_base joint")
    anchor = anchors[0]
    # 确认该关节确实是 world→elfin_base_link 的固定连接，而不是另一条机械连接。
    parent, child = anchor.find("parent"), anchor.find("child")
    if (anchor.get("type") != "fixed" or parent is None or child is None or
            parent.get("link") != "world" or child.get("link") != "elfin_base_link"):
        raise ValueError("Unexpected world_base structure")
    # 只允许单位锚定变换；若厂家模型在这里带平移/旋转，删除它会改变坐标含义，应拒绝。
    origin = anchor.find("origin")
    if origin is not None:
        for attr in ("xyz", "rpy"):
            values = [float(v) for v in origin.get(attr, "0 0 0").split()]
            if len(values) != 3 or any(abs(v) > 1e-12 or v != v for v in values):
                raise ValueError("world_base must be identity")
    # world 若带惯性/视觉等内容就不是可直接删除的空根链接。
    if list(worlds[0]):
        raise ValueError("Refusing to remove a world link with physical content")
    # 检查是否还有其他关节引用 world；只有确认锚点孤立，才允许移除。
    for joint in root.findall("joint"):
        if joint is anchor:
            continue
        for tag in ("parent", "child"):
            item = joint.find(tag)
            if item is not None and item.get("link") == "world":
                raise ValueError("Another joint references world")
    # 内部 elfin_base_link→elfin_base 必须存在且方向正确；其原始旋转值不改写。
    fixed = [x for x in root.findall("joint") if x.get("name") == "elfin_base_joint"]
    if len(fixed) != 1 or fixed[0].get("type") != "fixed":
        raise ValueError("Missing vendor base fixed joint")
    if (fixed[0].find("parent").get("link") != "elfin_base_link" or
            fixed[0].find("child").get("link") != "elfin_base"):
        raise ValueError("Unexpected vendor base link relation")
    # 所有局部结构检查通过后才修改内存树；厂家磁盘文件始终不变。
    root.remove(anchor)
    root.remove(worlds[0])
    # 删除锚点后重新检查根、重复子链接和可达性，不仅检查字符串是否成功删除。
    links = [x.get("name") for x in root.findall("link")]
    joints = root.findall("joint")
    children = [x.find("child").get("link") for x in joints]
    if len(set(links)) != len(links) or len(set(children)) != len(children):
        raise ValueError("Duplicate link or multiply-parented link")
    # 没有父关节的链接就是根；移动基座模型必须只有 elfin_base_link 一个根。
    if set(links) - set(children) != set(["elfin_base_link"]):
        raise ValueError("Model must have only elfin_base_link as root")
    # 从新根向下遍历有限轮数，检测断开的子树或不可达循环。
    reached = set(["elfin_base_link"])
    for unused in range(len(links)):
        for joint in joints:
            parent = joint.find("parent").get("link")
            child = joint.find("child").get("link")
            if parent not in links or child not in links:
                raise ValueError("Unknown link in joint")
            if parent in reached:
                reached.add(child)
    if reached != set(links):
        raise ValueError("Disconnected or cyclic model")
    return ET.tostring(root, encoding="utf-8")

# 命令行入口负责取输入和写输出；模型变换逻辑集中在 prepare_urdf 中。
def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--xacro")
    parser.add_argument("--urdf-file", help="Already expanded URDF, for offline checks")
    args = parser.parse_args()
    if bool(args.xacro) == bool(args.urdf_file):
        parser.error("Specify exactly one of --xacro or --urdf-file")
    if args.xacro:
        # 用参数列表调用 xacro，不把路径拼成 shell 命令；子进程失败直接中止。
        source = subprocess.check_output(["rosrun", "xacro", "xacro", "--inorder", args.xacro])
    else:
        with open(args.urdf_file, "rb") as stream:
            source = stream.read()
    result = prepare_urdf(source)
    # Python 3 写二进制 buffer；Python 2 的标准输出可直接接收 UTF-8 字节。
    getattr(sys.stdout, "buffer", sys.stdout).write(result)

if __name__ == "__main__":
    try:
        main()
    # 错误信息写 stderr，不能混进标准输出的 URDF，避免参数服务器加载损坏模型。
    except Exception as error:
        print("Model preparation failed: " + str(error), file=sys.stderr)
        sys.exit(1)
