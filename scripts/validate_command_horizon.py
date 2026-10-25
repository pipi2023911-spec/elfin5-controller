#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""离线单关节时序模型；不是ROS、完整QP、Gazebo或真实伺服。生产配置不变。"""
import argparse
import csv
import json
import math
from pathlib import Path


def sample(segment, time_ms):
    """三次Hermite与JTC结束后保持语义；位置rad、速度rad/s、时间ms。"""
    if segment is None:
        return 0., 0.
    start, end, p0, v0, p1, v1 = segment
    if time_ms > end:
        return p1, 0.
    if time_ms < start:
        return p0, 0.
    duration = (end-start)/1000.
    u = (time_ms-start)/(end-start)
    p = (2*u**3-3*u**2+1)*p0+(u**3-2*u**2+u)*duration*v0+(-2*u**3+3*u**2)*p1+(u**3-u**2)*duration*v1
    v = ((6*u**2-6*u)*p0+(-6*u**2+6*u)*p1)/duration+(3*u**2-4*u+1)*v0+(3*u**2-2*u)*v1
    return p, v


def simulate(horizon_ms, spin_ms=100, js_phase=7, spin_phase=37, cutoff_ms=None):
    dt_ms, js_ms, duration_ms = 20, 20, 6000
    vmax, amax, kp, target = .0785, .2512, 1.5, .5  # 当前候选参数的量级，不是实测规格。
    q = v = measured = previous_command = 0.
    segment = inbox = None
    trace, commands = [], []
    accepted = dropped = 0
    for t in range(duration_ms):
        q, v = sample(segment,t)
        if (t-js_phase) % js_ms == 0:
            measured = q
        if t % dt_ms == 0 and (cutoff_ms is None or t < cutoff_ms):
            requested = max(-vmax,min(vmax,kp*(target-measured)))
            dq = max(previous_command-amax*dt_ms/1000.,min(previous_command+amax*dt_ms/1000.,requested))
            commands.append((t,dq))
            previous_command = dq
            inbox = (t,measured+horizon_ms/1000.*dq,dq)
        if (t-spin_phase) % spin_ms == 0 and inbox is not None:
            stamp, endpoint, endpoint_v = inbox
            inbox = None
            start, end = t+1, stamp+horizon_ms
            if end <= start:
                dropped += 1
            else:
                p0, v0 = sample(segment,start)
                segment = (start,end,p0,v0,endpoint,endpoint_v)
                accepted += 1
        q, v = sample(segment,t+1)
        trace.append({'time_s':(t+1)/1000.,'position_rad':q,'velocity_rad_s':v})
    cruise = [r['velocity_rad_s'] for r in trace[1000:4000]]
    velocities = [r['velocity_rad_s'] for r in trace]
    full_accel = [abs(b-a)*1000. for a,b in zip(velocities,velocities[1:])]
    max_delta = max(abs(b[1]-a[1])/(dt_ms/1000.) for a,b in zip([(0,0.)]+commands,commands))
    metrics = {'horizon_ms':horizon_ms,'spin_ms':spin_ms,'js_phase':js_phase,'spin_phase':spin_phase,
               'cutoff_ms':cutoff_ms,'cruise_mean_velocity_rad_s':sum(cruise)/len(cruise),
               'cruise_peak_velocity_rad_s':max(map(abs,cruise)),
               'cruise_zero_fraction':sum(abs(x)<1e-6 for x in cruise)/len(cruise),
               'cruise_peak_fd_acceleration_rad_s2':max(abs(b-a)*1000. for a,b in zip(cruise,cruise[1:])),
               'startup_peak_fd_acceleration_rad_s2':max(full_accel[:1000]),
               'full_peak_fd_acceleration_rad_s2':max(full_accel),
               'max_qp_like_command_acceleration_rad_s2':max_delta,
               'accepted':accepted,'dropped':dropped}
    if cutoff_ms is not None:
        q_cut = trace[cutoff_ms-1]['position_rad']
        metrics['max_displacement_after_cutoff_rad'] = max(abs(r['position_rad']-q_cut) for r in trace[cutoff_ms:])
    return metrics, trace


def write_csv(path, rows):
    keys = list(dict.fromkeys(key for row in rows for key in row))
    with path.open('w',newline='',encoding='utf-8-sig') as file:
        writer = csv.DictWriter(file,fieldnames=keys)
        writer.writeheader(); writer.writerows(rows)


def run(output):
    output.mkdir(parents=True,exist_ok=False)
    rows, nominal = [], {}
    for h in (50,140):
        for js in (0,7,19):
            for spin in (0,37,99):
                metrics, trace = simulate(h,js_phase=js,spin_phase=spin)
                rows.append(metrics)
                if js==7 and spin==37:
                    nominal[h] = trace
                    write_csv(output/f'trace_h{h}.csv',trace)
        stopped, trace = simulate(h,cutoff_ms=4000)
        rows.append(stopped)
        write_csv(output/f'cutoff_h{h}.csv',trace)
    stalled, trace = simulate(140,spin_ms=200)
    rows.append(stalled)
    write_csv(output/'trace_stall200.csv',trace)
    write_csv(output/'comparison.csv',rows)
    checks = [{'dt_ms':dt,'factor':7,'horizon_ms':7*dt,'within_proposed_200ms':7*dt<=200}
              for dt in (20,25,30,200)]
    write_csv(output/'timer_horizon.csv',checks)
    (output/'summary.json').write_text(json.dumps({
        'cases':len(rows),'model':{'vmax_rad_s':.0785,'command_acceleration_rad_s2':.2512,
        'qp_like_period_ms':20,'joint_sample_ms':20,'rt_sample_ms':1,'target_rad':.5,
        'statistics_window_s':[1,4]},
        'limits':['单关节理想伺服、简化位置反馈，不运行实际QP或网关，不包含真实停止适配器。',
                  '所有加速度和停发位移均为模型结果，不是硬件保护指标。',
                  '只改变实验horizon；未修改生产YAML、5mm容差或关节限制。'],
        'rows':rows,'timer_checks':checks},ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(2,1,figsize=(10,7))
    for h, trace in nominal.items():
        ts = [r['time_s'] for r in trace]
        vs = [r['velocity_rad_s'] for r in trace]
        axes[0].plot(ts,vs,label=f'h={h} ms')
        axes[1].plot(ts[1:],[1000*(b-a) for a,b in zip(vs,vs[1:])],label=f'h={h} ms')
    for ax in axes:
        ax.set_xlim(1,1.5);ax.legend();ax.grid(alpha=.25)
    axes[0].set_ylabel('Ideal joint velocity (rad/s)')
    axes[1].set(xlabel='Model time (s)',ylabel='Finite-difference acceleration (rad/s2)')
    fig.tight_layout();fig.savefig(output/'horizon_comparison.png',dpi=160);plt.close(fig)
    print(f'已完成{len(rows)}组离线时序案例：{output}；未运行ROS或设备')
    return output


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True,help='新建输出目录，不覆盖已有结果')
    run(parser.parse_args().output)
