#!/usr/bin/env python
# -*- coding: utf-8 -*-
# 【文件作用】离线核验厂家完整 URDF；真机启动直接使用 bringup 的 /robot_description。
# 【输入】--xacro 厂家模型路径，或 --urdf-file 已展开的 URDF（二选一）。
# 【输出】核验成功后标准输出写原始 XML，不删除或重写厂家链接及关节。
# 【修改范围】不修改模型；检查 world_base 固定单位变换、内部基座关系和树结构。
# 【失败处理】模型结构、根节点或连接关系不符时返回非零退出码，错误写到标准错误。
# 本脚本不编辑厂家文件、不发布 TF、不连接硬件；绝对系→world 的动态变换由定位链或监视节点提供。
# 兼容 Python 2.7 与 Python 3；文件头编码声明和 __future__ 导入的顺序不能破坏。

"""离线检查并保留厂家完整 URDF（兼容 Python 2.7/3）。"""
from __future__ import print_function
import argparse
import subprocess
import sys
import xml.etree.ElementTree as ET

# 核心纯函数：接收 XML 字节或文本，返回原始 XML 字节；独立于 ROS，便于离线测试。
def prepare_urdf(xml):
    root = ET.fromstring(xml)
    if root.tag != "robot":
        raise ValueError("输入必须是 robot 类型的 URDF")
    # 按准确名称找到锚定世界的两个元素，数量必须唯一，确保坐标契约明确。
    worlds = [x for x in root.findall("link") if x.get("name") == "world"]
    anchors = [x for x in root.findall("joint") if x.get("name") == "world_base"]
    if len(worlds) != 1 or len(anchors) != 1:
        raise ValueError("模型必须恰好包含一个 world 链接和 world_base 关节")
    anchor = anchors[0]
    # 确认该关节确实是 world→elfin_base_link 的固定连接，而不是另一条机械连接。
    parent, child = anchor.find("parent"), anchor.find("child")
    if (anchor.get("type") != "fixed" or parent is None or child is None or
            parent.get("link") != "world" or child.get("link") != "elfin_base_link"):
        raise ValueError("world_base 必须是 world 到 elfin_base_link 的固定关节")
    # 只允许单位锚定变换；若厂家模型在这里带平移/旋转，就不能直接把基座位姿赋给模型根，应拒绝。
    origin = anchor.find("origin")
    if origin is not None:
        for attr in ("xyz", "rpy"):
            values = [float(v) for v in origin.get(attr, "0 0 0").split()]
            if len(values) != 3 or any(abs(v) > 1e-12 or v != v for v in values):
                raise ValueError("world_base 必须是单位变换")
    # 内部 elfin_base_link→elfin_base 必须存在且方向正确；其原始旋转值不改写。
    fixed = [x for x in root.findall("joint") if x.get("name") == "elfin_base_joint"]
    if len(fixed) != 1 or fixed[0].get("type") != "fixed":
        raise ValueError("缺少厂家内部基座固定关节")
    if (fixed[0].find("parent").get("link") != "elfin_base_link" or
            fixed[0].find("child").get("link") != "elfin_base"):
        raise ValueError("厂家内部基座链接关系不符")
    # 保留所有模型元素，只核验树结构；不假定内部固定旋转为某个角度。
    links = [x.get("name") for x in root.findall("link")]
    joints = root.findall("joint")
    children = [x.find("child").get("link") for x in joints]
    if len(set(links)) != len(links) or len(set(children)) != len(children):
        raise ValueError("模型存在重复链接或一个链接有多个父关节")
    # 没有父关节的链接就是根；完整厂家模型必须只有 world 一个根。
    if set(links) - set(children) != set(["world"]):
        raise ValueError("完整厂家模型必须仅以 world 为根")
    # 从原模型根向下遍历有限轮数，检测断开的子树或不可达循环。
    reached = set(["world"])
    for unused in range(len(links)):
        for joint in joints:
            parent = joint.find("parent").get("link")
            child = joint.find("child").get("link")
            if parent not in links or child not in links:
                raise ValueError("关节引用了未知链接")
            if parent in reached:
                reached.add(child)
    if reached != set(links):
        raise ValueError("模型存在断开或循环结构")
    return xml.encode("utf-8") if not isinstance(xml, bytes) else xml

# 命令行入口负责取输入和写输出；模型核验逻辑集中在 prepare_urdf 中。
def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--xacro")
    parser.add_argument("--urdf-file", help="已展开的 URDF，用于离线核验")
    args = parser.parse_args()
    if bool(args.xacro) == bool(args.urdf_file):
        parser.error("必须且只能指定 --xacro 或 --urdf-file 之一")
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
        print("模型核验失败：" + str(error), file=sys.stderr)
        sys.exit(1)
