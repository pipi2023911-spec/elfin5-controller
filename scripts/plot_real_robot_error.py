#!/usr/bin/env python
# -*- coding: utf-8 -*-
# 【文件作用】从真机录包生成末端误差模长图和六关节有符号过程误差图。
# 【输入】ROS bag 路径、全新的输出目录，以及可选话题名/容差/ACTIVE阶段过滤参数。
# 【末端数据】读取 monitor/tracking_state 中的 E(t) 和有效标志，不绘制XYZ分量。
# 【关节数据】同一 JTC state 的 desired−actual；按关节名称配对，不用未来QP端点。
# 【单位】CSV内关节角度为rad；图和关节统计为°；末端图和统计为mm。
# 【时间】横轴统一使用录包接收时间相对起点；源测量时间仍存CSV，用于评估延迟/同步。
# 【输出】两类PNG、原始配对CSV、事件CSV及统计JSON；绝不覆盖已有输出目录。
# 【无效数据】以NaN断开曲线，不补零、不做角度取模；低频采样不能证明不存在高频峰值。
# 此脚本离线读取数据，不发布ROS消息，也不会使机械臂运动。

"""Plot TCP error norm and six signed joint tracking errors from a real ROS bag."""
from __future__ import print_function
import argparse
import csv
import json
import math
import os
import sys

# 本项目六个有限位转动关节的标准输出顺序；录包内原数组顺序可以不同。
JOINTS = ["elfin_joint%d" % i for i in range(1, 7)]

# 使用 isnan/isinf 兼容 Python 2.7，排除会污染曲线和统计的非有限数。
def finite(value):
    return not math.isnan(value) and not math.isinf(value)

# 关节数组必须数量一致、名字唯一且覆盖预期六轴；先验证再逐轴相减。
def joint_errors(names, desired, actual, expected=JOINTS):
    if (len(names) != len(expected) or len(set(names)) != len(names) or
            len(desired) != len(names) or len(actual) != len(names) or
            set(names) != set(expected)):
        raise ValueError("Invalid joint names or array sizes")
    # 建立名称→原数组索引，避免把J1的期望角减去J2的实际角。
    mapping = dict((name, i) for i, name in enumerate(names))
    result = []
    for name in expected:
        index = mapping[name]
        if not finite(desired[index]) or not finite(actual[index]):
            raise ValueError("Non-finite joint position")
        # 保留误差符号与原角度分支；此处不 wrap 到 ±π，防止掩盖零位/跳变问题。
        result.append(desired[index] - actual[index])
    return result

# 统计单一标量序列；调用方已把无效/不统计阶段的误差填为NaN。
# eligible 只定义参与覆盖率与时长统计的阶段，避免把非ACTIVE时间计入丢数时间。
def series_stats(times, values, max_gap, threshold=None, eligible=None):
    eligible = eligible if eligible is not None else [True]*len(times)
    # 样本RMSE/P95只使用有限误差值；这不是按时间积分加权的RMSE。
    samples = [v for v in values if finite(v)]
    absolute = sorted(abs(v) for v in samples)
    # 计算参与统计的正向时间间隔；阶段边界和时间回退不会产生负时长。
    periods = [times[i]-times[i-1] for i in range(1,len(times))
               if times[i]>times[i-1] and eligible[i] and eligible[i-1]]
    duration = sum(periods)
    ordered_periods = sorted(periods)
    covered, exceeded = 0.0, 0.0
    for i in range(1, len(times)):
        dt = times[i] - times[i - 1]
        # 只有相邻两点都有效且间隔不超过max_gap，才累计可观测时长。
        if (eligible[i] and eligible[i-1] and 0 < dt <= max_gap and
                finite(values[i-1]) and finite(values[i])):
            covered += dt
            # Piecewise-constant duration estimate, explicitly tied to sample rate.
            if threshold is not None and abs(values[i-1]) > threshold:
                exceeded += dt
    return {
        "samples": len(samples),
        "sample_rmse": math.sqrt(sum(v*v for v in samples)/len(samples)) if samples else None,
        "max_abs": max(absolute) if samples else None,
        # 采用最近秩法取绝对误差的95百分位；不把正负值直接排序后取P95。
        "p95_abs": absolute[int(math.ceil(0.95*len(absolute)))-1] if samples else None,
        "covered_seconds": covered,
        "coverage": covered/duration if duration else 0.0,
        "eligible_duration_seconds": duration,
        "threshold": threshold,
        # 这是接收时间间隔的中位数对应频率，不是驱动器实际伺服周期。
        "observed_receipt_hz_median": 1.0/ordered_periods[len(periods)//2] if periods else None,
        "max_observed_gap_seconds": max(periods) if periods else None,
        "exceeded_seconds_estimate": exceeded if threshold is not None else None
    }

# 跨大间隔或时间不递增时插入NaN绘图点，让Matplotlib断线；不改写原始样本值。
def with_gaps(times, values, max_gap):
    x, y = [], []
    for i, (t, value) in enumerate(zip(times, values)):
        if i and (t <= times[i-1] or t-times[i-1] > max_gap):
            x.append(t); y.append(float("nan"))
        x.append(t); y.append(value)
    return x, y

# Python2的csv使用二进制文件，Python3关闭自动换行转换，避免Windows出现空白行。
def write_csv(path, header, rows):
    if sys.version_info[0] < 3:
        stream = open(path, "wb")
    else:
        stream = open(path, "w", newline="")
    with stream:
        writer = csv.writer(stream)
        writer.writerow(header)
        writer.writerows(rows)

# 命令行参数解析与录包处理入口；rosbag/matplotlib延迟导入，便于纯函数离线测试。
def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag")
    parser.add_argument("--output", required=True, help="New output directory (never overwrite)")
    parser.add_argument("--controller-topic", default="/elfin_arm_controller/state")
    parser.add_argument("--tracking-topic", default="/elfin5_real/monitor/tracking_state")
    parser.add_argument("--bridge-topic", default="/elfin5_real/bridge/state")
    parser.add_argument("--target-topic", default="/target_point")
    parser.add_argument("--base-topic", default="/base_pose")
    parser.add_argument("--max-gap", type=float, default=0.3)
    parser.add_argument("--tcp-tolerance-mm", type=float)
    parser.add_argument("--joint-tolerance-deg", nargs=6, type=float)
    # 默认显示全部阶段；指定此选项时只保留ACTIVE误差，其他阶段以无效段分隔。
    parser.add_argument("--active-only", action="store_true",
                        help="Statistics/plots include ACTIVE intervals only; default includes all phases")
    args = parser.parse_args()
    for value in [args.max_gap] + ([args.tcp_tolerance_mm] if args.tcp_tolerance_mm is not None else []) + (args.joint_tolerance_deg or []):
        if not finite(value) or value <= 0:
            parser.error("Gap and optional tolerances must be finite and positive")
    # 拒绝覆盖已有实验分析目录，防止同名输出悄悄替换上一轮结果。
    if os.path.exists(args.output):
        parser.error("Output directory already exists")
    import rosbag
    import matplotlib
    # 使用无窗口绘图后端，现场无桌面环境时也可以导出PNG。
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    # 三个独立记录集合：末端误差、逐轴参考/反馈/误差，以及状态/目标/基座事件。
    tcp, joint_rows, events = [], [], []
    phase = "UNKNOWN"
    last_target = None
    last_base = None
    moving = False
    last_stamps = {}
    topics = [args.controller_topic, args.tracking_topic, args.bridge_topic,
              args.target_topic, args.base_topic]
    with rosbag.Bag(args.bag, "r") as bag:
        # 按录包顺序处理；receipt是录包接收时间，不能等同于消息的测量时间。
        for topic, msg, receipt in bag.read_messages(topics=topics):
            # 只有网关状态前缀改变才记事件；同时维护后续样本所属的ACTIVE/其他阶段。
            if topic == args.bridge_topic:
                new = msg.data.split(":")[0]
                if new != phase:
                    events.append((receipt.to_sec(), new))
                    phase = new
                continue
            # 目标数值发生改变才记事件；仅时间戳变化的固定目标心跳不产生新事件。
            if topic == args.target_topic:
                point = (msg.point.x, msg.point.y, msg.point.z)
                if point != last_target:
                    events.append((receipt.to_sec(), "target"))
                    last_target = point
                continue
            # 由相邻原始基座位姿估计运动/静止事件，阈值仅用于图表提示，不参与控制或误差计算。
            if topic == args.base_topic:
                t = msg.header.stamp.to_sec()
                p = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
                q = (msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w)
                if last_base and t > last_base[0]:
                    dt = t - last_base[0]
                    speed = math.sqrt(sum((a-b)**2 for a,b in zip(p,last_base[1])))/dt
                    # 四元数q与−q代表同一姿态，因此使用点积绝对值；夹紧到范围内避免acos数值越界。
                    dot = min(1.0, max(-1.0, abs(sum(a*b for a,b in zip(q,last_base[2])))))
                    omega = 2*math.acos(dot)/dt
                    now_moving = speed > 0.002 or omega > 0.01
                    if now_moving != moving:
                        events.append((receipt.to_sec(), "base_moving" if now_moving else "base_still"))
                        moving = now_moving
                last_base = (t,p,q)
                continue
            stamp = msg.header.stamp.to_sec()
            receipt_s = receipt.to_sec()
            # 源时间须非零且严格递增；重复或回退样本不能被画成正常误差。
            monotonic = stamp > 0 and stamp > last_stamps.get(topic, 0)
            if monotonic:
                last_stamps[topic] = stamp
            # 同时检查源时间到录包接收时间的延迟；这依赖参与设备时钟已同步。
            time_ok = monotonic and -0.01 <= receipt_s-stamp <= args.max_gap
            phase_ok = not args.active_only or phase == "ACTIVE"
            if topic == args.tracking_topic:
                for status in msg.status:
                    if status.name != "elfin5_real_tracking":
                        continue
                    # 将诊断键值转为字典，只识别monitor定义的跟踪状态，不混入其他诊断。
                    fields = dict((item.key, item.value) for item in status.values)
                    valid = time_ok and phase_ok and fields.get("valid") == "1"
                    value = float(fields.get("position_error_m", "nan")) * 1000
                    # 末端行：接收时间、测量时间、mm误差、阶段、参与计算的基座/目标时间。
                    tcp.append((receipt_s, stamp, value if valid else float("nan"), phase,
                                fields.get("base_stamp",""), fields.get("target_stamp","")))
            elif topic == args.controller_topic:
                try:
                    # 参考和实际位置来自同一state，配对后才得到执行过程误差。
                    errors = joint_errors(msg.joint_names, msg.desired.positions, msg.actual.positions)
                    order = [msg.joint_names.index(name) for name in JOINTS]
                    desired = [msg.desired.positions[i] for i in order]
                    actual = [msg.actual.positions[i] for i in order]
                    # 同时保留控制器自己的error字段，便于核查其角差处理是否与直接相减一致。
                    reported = ([msg.error.positions[i] for i in order]
                                if len(msg.error.positions) == 6 else [float("nan")]*6)
                except (ValueError, IndexError):
                    errors = desired = actual = reported = [float("nan")]*6
                if not time_ok or not phase_ok:
                    errors = [float("nan")]*6
                # 关节行布局：3个元数据列 + 6期望 + 6实际 + 6计算误差 + 6控制器原误差。
                # 后续绘图从索引15读取计算误差，修改列布局时必须同步修改导出与绘图。
                joint_rows.append([receipt_s, stamp, phase] + desired + actual + errors + reported)
    # 无匹配数据时明确报错，避免输出看似成功但实际上没有观测的图。
    if not tcp and not joint_rows:
        raise ValueError("No matching tracking/controller state messages in bag")
    # 确认输入至少含目标消息后创建新目录；只向此目录写分析产物。
    os.makedirs(args.output)
    write_csv(os.path.join(args.output,"tcp_error.csv"),
              ["receipt_s","stamp_s","error_mm","phase","base_stamp_s","target_stamp_s"],tcp)
    write_csv(os.path.join(args.output,"joint_error.csv"),
              ["receipt_s","stamp_s","phase"]+
              [prefix+name for prefix in ("desired_rad_","actual_rad_","error_rad_","controller_error_rad_") for name in JOINTS],
              joint_rows)
    write_csv(os.path.join(args.output,"events.csv"),["receipt_s","event"],events)
    # 两类图使用同一个时间零点，便于对照任务误差与关节执行误差。
    all_times = [r[0] for r in tcp]+[r[0] for r in joint_rows]
    origin = min(all_times)
    metadata = {"time_axis": "bag receipt seconds relative to first sample; measurement stamps retained in CSV",
                "phase_filter": "ACTIVE" if args.active_only else "all",
                "max_gap_s": args.max_gap, "tcp_unit":"mm","joint_unit":"degree",
                "angular_difference":"signed direct difference; bounded joints, common angle branch required",
                "bag":os.path.abspath(args.bag)}
    # 统一网格、时间轴和事件竖线；图上最多前50个事件，完整列表保存在events.csv。
    def decorate(axis):
        axis.grid(True, alpha=0.3)
        axis.set_xlabel("Time (s)")
        for t, label in events[:50]:
            axis.axvline(t-origin, color="grey", alpha=0.18, linewidth=0.7)
        # Complete event labels are in events.csv; no invented motion events.
    # 末端只画一条E(t)；仅在用户提供容差时绘制阈值线，不猜测硬件精度要求。
    if tcp:
        times = [r[0]-origin for r in tcp]; values = [r[2] for r in tcp]
        fig, ax = plt.subplots(figsize=(10,4))
        ax.plot(*with_gaps(times,values,args.max_gap), color="tab:blue")
        if args.tcp_tolerance_mm is not None:
            ax.axhline(args.tcp_tolerance_mm,color="red",linestyle="--")
        ax.set_ylabel("TCP error norm (mm)"); ax.set_title("World-frame TCP tracking error")
        decorate(ax);fig.tight_layout()
        fig.savefig(os.path.join(args.output,"tcp_error.png"),dpi=180);plt.close(fig)
        metadata["tcp"]=series_stats(times,values,args.max_gap,args.tcp_tolerance_mm,
            [not args.active_only or r[3]=="ACTIVE" for r in tcp])
    # 六轴采用3×2子图并共享时间轴，保留每轴符号与零线，便于比较持续偏差和瞬态。
    if joint_rows:
        times=[r[0]-origin for r in joint_rows]
        fig, axes=plt.subplots(3,2,sharex=True,figsize=(12,9))
        metadata["joints"]={}
        for i,ax in enumerate(axes.flat):
            # rad转为°仅发生在展示/统计阶段，CSV保留原始rad便于复算。
            values=[r[15+i]*180/math.pi for r in joint_rows]
            ax.plot(*with_gaps(times,values,args.max_gap),linewidth=0.9)
            ax.axhline(0,color="black",linewidth=0.6);ax.set_ylabel("Error (deg)");ax.set_title(JOINTS[i])
            threshold=args.joint_tolerance_deg[i] if args.joint_tolerance_deg else None
            if threshold is not None:
                for sign in (-1,1):ax.axhline(sign*threshold,color="red",linestyle="--")
            decorate(ax)
            metadata["joints"][JOINTS[i]]=series_stats(times,values,args.max_gap,threshold,
                [not args.active_only or r[2]=="ACTIVE" for r in joint_rows])
        fig.tight_layout();fig.savefig(os.path.join(args.output,"joint_tracking_error.png"),dpi=180);plt.close(fig)
    # 统计中的缺失结果用None/null表示；禁止把NaN写成非标准JSON数值。
    with open(os.path.join(args.output,"statistics.json"),"w") as stream:
        json.dump(metadata,stream,indent=2,allow_nan=False)
    print("Saved error figures, raw paired samples, events and statistics to "+args.output)

if __name__ == "__main__":
    main()
