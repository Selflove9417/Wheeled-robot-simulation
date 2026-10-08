#!/usr/bin/env python3
"""只读绘制控制器CSV：运动响应与髋膝参考/反馈合并在一张图中。

python3 plot_jump_log.py [CSV] [-o PNG]
轮底净空为控制器几何估计；状态切换不代表真实物理接触事件。
actual_tau_*是Effort发布前命令，不是全程实际驱动力矩。
仅effort_mode_active=1且leg_mode_switch_pending=0时绘制Effort命令；
模式或切换信息缺失时保留NaN，不根据运动状态推断接口所有权。
"""
import argparse
import csv
import json
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Patch

plt.rcParams.update({
    'font.sans-serif': ['Noto Sans CJK JP', 'Noto Sans CJK SC',
                        'WenQuanYi Micro Hei', 'DejaVu Sans'],
    'axes.unicode_minus': False, 'font.size': 9,
    'axes.spines.top': False, 'axes.spines.right': False,
    'savefig.facecolor': 'white',
})
# State/subphase values from bbot_velocity_jump_controller.cpp.
PHASES = {
    0: ('BALANCE', '#eeeeee'), 8: ('PRE_JUMP', '#fff0ce'),
    1: ('SQUAT', '#ffe0bd'), 2: ('THRUST', '#ffe589'),
    4: ('BUFFER', '#dce5ed'), 5: ('RECOVERY', '#d5ebd6'),
    6: ('STANDUP', '#f5c9c9'), 7: ('EMERGENCY', '#f39f9f'),
}
AIR = {0: ('ARREST', '#e1cef1'), 1: ('TUCK', '#c9dfff'),
       2: ('EXTEND', '#bce8e4'), 3: ('PROTECTIVE', '#f6b3a9')}
COLORS = {'hip': '#1b6ca8', 'knee': '#c44832'}


def load_csv(filepath):
    """Parse mixed strings/numbers; missing numeric entries remain NaN."""
    with Path(filepath).open(newline='') as f:
        reader = csv.DictReader(f)
        names = reader.fieldnames or []
        rows = list(reader)
    if not rows or not {'timestamp', 'state'} <= set(names):
        raise ValueError('日志为空或缺少 timestamp/state 字段')
    if len(set(names)) != len(names) or any(None in r for r in rows):
        raise ValueError('CSV列重复或行宽不一致；请先确认日志完整性')
    data = {}
    for name in names:
        values = []
        for r in rows:
            try:
                values.append(float(r.get(name, '')))
            except (ValueError, TypeError):
                values.append(np.nan)
        data[name] = np.array(values)
    t = data['timestamp']
    if not np.isfinite(t).all() or np.any(np.diff(t) <= 0):
        raise ValueError('时间戳缺失、重复或回退，拒绝连接为连续曲线')
    print(f'[INFO] 加载 {len(t)} 行，时长 {t[-1]-t[0]:.3f}s')
    return data


def field(data, name):
    return data.get(name, np.full(len(data['timestamp']), np.nan)).copy()


def valid_field(data, name, validity):
    result = field(data, name)
    # Missing validity is unknown, never assume a valid COM estimate.
    result[field(data, validity) != 1] = np.nan
    return result


def effort_command(data, part):
    """Use explicit local mode/switch flags, never infer mode from jump phases.

    These flags describe the node's recorded mode, not independent controller
    manager readback or proof that physics consumed this command.
    """
    name = f'actual_tau_{part}_left'
    if name not in data:
        name = f'{part}_cmd_left'
    result = field(data, name)
    valid = ((field(data, 'effort_mode_active') == 1) &
             (field(data, 'leg_mode_switch_pending') == 0))
    result[~valid | ~np.isfinite(result)] = np.nan
    return result


def compute_jump_metrics(data):
    """No fabricated ballistic height, radius, zero fill or clipping."""
    t = data['timestamp'] - data['timestamp'][0]
    com = valid_field(data, 'com_world_z', 'com_velocity_valid')
    clearance = field(data, 'wheel_clearance')
    clearance[data['state'] != 3] = np.nan  # only updated flight diagnostic
    return t, com, clearance


def load_native_axle(data, geometry_path, metrics_path):
    """Project paired native wheel poses; reuse an audited physical contact event.

    Never use controller x/odometry or integrate wheel speed as world position.
    The caller supplies geometry and metrics from the same recorded trial.
    """
    with metrics_path.open() as f:
        metrics = json.load(f)
    if metrics.get('run') != geometry_path.parent.name:
        raise ValueError('geometry与验收metrics的试次名称不一致')
    contact = float(metrics['first_touch_s'])
    timestamps = data['timestamp']
    if not np.isfinite(contact) or not timestamps[0] <= contact <= timestamps[-1]:
        raise ValueError('真实首触时刻不在控制CSV记录区间内')
    index = np.searchsorted(timestamps, contact, side='right') - 1
    direction = np.array([field(data, 'jump_forward_axis_x')[index],
                          field(data, 'jump_forward_axis_y')[index]])
    if (field(data, 'jump_forward_axis_valid')[index] != 1 or
            not np.isfinite(direction).all() or
            abs(np.linalg.norm(direction) - 1) > 1e-4):
        raise ValueError('缺少有效的起跳世界前向轴，拒绝猜测位移方向')
    with geometry_path.open(newline='') as f:
        reader = csv.DictReader(f)
        required = {'sim_time_ns', 'physics_iteration', 'dt_ns', 'frame_valid',
                    'left_wheel_x', 'left_wheel_y', 'right_wheel_x', 'right_wheel_y'}
        if not required <= set(reader.fieldnames or []):
            raise ValueError('geometry缺少原生轮轴位姿或时钟字段')
        rows = list(reader)
    ns = np.array([int(r['sim_time_ns']) for r in rows], dtype=np.int64)
    iteration = np.array([int(r['physics_iteration']) for r in rows], dtype=np.int64)
    dt = np.array([int(r['dt_ns']) for r in rows], dtype=np.int64)
    if not len(ns) or np.any(np.diff(ns) <= 0):
        raise ValueError('原生geometry时间戳为空、重复或回退')
    poses = np.array([[float(r.get(k) or 'nan') for k in
                      ('left_wheel_x', 'left_wheel_y', 'right_wheel_x', 'right_wheel_y')]
                     for r in rows])
    valid = np.array([r['frame_valid'] == '1' for r in rows])
    valid &= np.isfinite(poses).all(axis=1) & (dt > 0)
    absolute = (.5 * (poses[:, :2] + poses[:, 2:])) @ direction
    absolute[~valid] = np.nan
    hit = np.flatnonzero(ns == round(contact * 1e9))
    if len(hit) != 1 or not np.isfinite(absolute[hit[0]]):
        raise ValueError('真实首触缺少同戳有效轮轴位姿，拒绝替代锁存')
    anchor = absolute[hit[0]]
    relative = absolute - anchor
    relative[ns < ns[hit[0]]] = np.nan
    time = ns / 1e9 - timestamps[0]
    # Insert NaN separators at dropped physics frames, retaining both endpoints.
    gaps = np.flatnonzero((np.diff(ns) != dt[1:]) | (np.diff(iteration) != 1)) + 1
    return {'time': np.insert(time, gaps, np.nan),
            'absolute': np.insert(absolute, gaps, np.nan),
            'relative': np.insert(relative, gaps, np.nan),
            'contact': contact - timestamps[0]}


def phase_at(data, i):
    state = data['state'][i]
    if state == 3:
        subphase = data['flight_subphase'][i] if 'flight_subphase' in data else np.nan
        return AIR.get(subphase, ('FLIGHT', '#e1cef1'))
    return PHASES.get(state, ('UNKNOWN', '#dddddd'))


def phase_segments(data, t):
    keys = [phase_at(data, i) for i in range(len(t))]
    bounds = [0] + [i for i in range(1, len(t)) if keys[i] != keys[i-1]] + [len(t)]
    for a, b in zip(bounds, bounds[1:]):
        yield t[a], (t[b] if b < len(t) else t[-1]), keys[a]


def flight_windows(data):
    """All contiguous FLIGHT episodes, including protective/failure episodes."""
    flight = data['state'] == 3
    changes = np.diff(np.r_[False, flight, False].astype(int))
    return list(zip(np.where(changes == 1)[0], np.where(changes == -1)[0]))


def draw(ax, t, y, label, **kwargs):
    if np.isfinite(y).any():
        ax.plot(t, y, label=label, linewidth=1.25, **kwargs)


def finish_axes(axes):
    for ax in axes:
        ax.grid(alpha=.20, linestyle='--')
        ax.margins(x=0)
        if ax.get_legend_handles_labels()[0]:
            ax.legend(loc='upper left', fontsize=8, framealpha=.88)
        else:
            ax.text(.5, .5, '缺少有效记录', ha='center', va='center',
                    transform=ax.transAxes, color='#777777')


def add_phases(fig, timeline, axes, data, t):
    seen = {}
    for a, b, (name, color) in phase_segments(data, t):
        timeline.axvspan(a, b, color=color)
        seen[name] = color
        if name not in ('BALANCE', 'UNKNOWN'):
            for ax in axes:
                ax.axvspan(a, b, color=color, alpha=.15, zorder=0)
    timeline.set_yticks([])
    timeline.set_ylabel('阶段', rotation=0, labelpad=20)
    timeline.tick_params(axis='x', labelbottom=False)
    timeline.legend(handles=[Patch(facecolor=c, label=n) for n, c in seen.items()],
                    loc='lower left', bbox_to_anchor=(0, 1), ncol=9,
                    fontsize=8, frameon=False)


def plot_jump_performance(data, output_path, native_axle=None, time_window=None):
    """Preserve five response panels and add separate hip/knee angle panels."""
    t, com, clearance = compute_jump_metrics(data)
    panels = 8 if native_axle is not None else 7
    fig, axes = plt.subplots(panels, 1, figsize=(14, 17 if panels == 7 else 20), sharex=True,
                             layout='constrained')
    fig.suptitle('BBot 双轮腿机器人跳跃响应', fontsize=15, fontweight='bold')
    ax = axes[0]
    draw(ax, t, com, '质心高度估计 · com_world_z', color='#1769aa')
    draw(ax, t, clearance, '轮底净空估计 · wheel_clearance', color='#e17822')
    ax.set_ylabel('高度 [m]')

    ax = axes[1]
    draw(ax, t, valid_field(data, 'com_world_vz', 'com_velocity_valid'),
         '质心竖直速度估计', color='#8845aa')
    draw(ax, t, field(data, 'target_takeoff_velocity'),
         '目标离地速度', color='#388b61', linestyle='--')
    ax.set_ylabel('竖直速度 [m/s]')
    count_axis = ax.twinx()
    count_axis.plot(t, field(data, 'airborne_confidence'), color='#009688',
                    linewidth=.9, alpha=.6, label='离地确认计数')
    count_axis.plot(t, field(data, 'wheels_airborne'), color='#e17822',
                    linewidth=.9, linestyle=':', label='控制器离地标志（0/1）')
    count_axis.set_ylabel('计数 / 标志')

    ax = axes[2]
    for part, label in [('hip', '左髋'), ('knee', '左膝')]:
        draw(ax, t, effort_command(data, part), label+'Effort命令', color=COLORS[part])
    ax.set_title('髋膝Effort力矩命令', loc='left', fontsize=10)
    ax.set_ylabel('Effort命令 [N·m]')
    ax.text(.99, .95, '仅绘制日志确认的Effort模式且无切换待完成的样本\n'
            'Position / 切换中 / 模式未知：NaN断线；非物理力矩测量',
            ha='right', va='top', transform=ax.transAxes, fontsize=8,
            bbox=dict(facecolor='white', edgecolor='none', alpha=.85))

    for ax, part, joint_label in [(axes[3], 'hip', '髋'), (axes[4], 'knee', '膝')]:
        for side, side_label, color in [('left', '左', '#1b6ca8'),
                                         ('right', '右', '#c44832')]:
            draw(ax, t, field(data, f'{part}_pos_cmd_{side}'),
                 f'{side_label}{joint_label}参考', color=color, linestyle='--')
            draw(ax, t, field(data, f'{part}_pos_{side}'),
                 f'{side_label}{joint_label}实测', color=color)
        ax.set_title(f'{joint_label}关节参考与实测角度', loc='left', fontsize=10)
        ax.set_ylabel('关节角度 [rad]')
    axes[3].text(.99, .95, '虚线：控制节点角度参考；实线：joint_states反馈\n'
                 'Effort模式下参考用于软件跟踪，不代表发布了Position命令',
                 ha='right', va='top', transform=axes[3].transAxes, fontsize=8,
                 bbox=dict(facecolor='white', edgecolor='none', alpha=.85))

    ax = axes[5]
    draw(ax, t, np.degrees(field(data, 'pitch')), 'pitch（正=前倾）', color='#222222')
    ax.set_ylabel('俯仰角 [°]')
    rate_axis = ax.twinx()
    draw(rate_axis, t, field(data, 'pitch_rate'), 'pitch_rate（滤波）', color='#8845aa')
    rate_axis.set_ylabel('俯仰角速度 [rad/s]', color='#8845aa')

    ax = axes[6]
    draw(ax, t, field(data, 'left_wheel_vel'), '左轮编码器角速度', color='#b44b42')
    draw(ax, t, field(data, 'right_wheel_vel'), '右轮编码器角速度', color='#388b61', linestyle='--')
    ax.set_ylabel('轮角速度 [rad/s]')
    command_axis = ax.twinx()
    draw(command_axis, t, field(data, 'cmd_x'), '发布线速度命令 cmd_x', color='#1769aa')
    command_axis.set_ylabel('线速度命令 [m/s]', color='#1769aa')
    twins = [(axes[1], count_axis), (axes[5], rate_axis), (axes[6], command_axis)]
    if native_axle is not None:
        ax = axes[7]
        draw(ax, native_axle['time'], native_axle['absolute'],
             '双轮轴中点世界前向位置（原生位姿投影）', color='#1769aa')
        ax.set_ylabel('世界前向位置 [m]', color='#1769aa')
        displacement_axis = ax.twinx()
        draw(displacement_axis, native_axle['time'], 100 * native_axle['relative'],
             '真实首触后相对位移（负=后退）', color='#8845aa')
        displacement_axis.set_ylabel('首触后位移 [cm]', color='#8845aa')
        ax.set_title('原生轮轴水平位置与真实首触后位移', loc='left', fontsize=10)
        twins.append((ax, displacement_axis))
        for panel in axes:
            panel.axvline(native_axle['contact'], color='#555555', linestyle=':',
                          linewidth=.9)
        ax.text(.99, .05, '竖虚线：已有原生接触验收的首触时刻；位移直接来自轮位姿，不积分轮速',
                ha='right', va='bottom', transform=ax.transAxes, fontsize=8)
    axes[-1].set_xlabel('相对日志首帧时间 [s]')

    # Keep stage backgrounds, with one shared legend instead of repeated labels.
    seen = {}
    for start, end, (label, color) in phase_segments(data, t):
        if label in ('BALANCE', 'UNKNOWN'):
            continue
        seen[label] = color
        for ax in axes:
            ax.axvspan(start, end, color=color, alpha=.20, zorder=0)
    finish_axes(axes)
    for ax, twin in twins:
        lines, labels = ax.get_legend_handles_labels()
        extra, extra_labels = twin.get_legend_handles_labels()
        ax.legend(lines+extra, labels+extra_labels, loc='upper right',
                  fontsize=9, framealpha=.9, ncol=2)
    for ax in axes:
        # A full-width zero guide in the Effort panel resembles a zero command
        # during Position support, where the command curve is intentionally absent.
        if ax is not axes[2]:
            ax.axhline(0, color='#888888', linewidth=.6, alpha=.5)
    if len(t) > 1:
        axes[0].set_xlim(t[0], t[-1])
    if time_window is not None:
        if not np.isfinite(time_window).all() or time_window[0] >= time_window[1]:
            raise ValueError('显示时间窗口必须有限且起点小于终点')
        axes[0].set_xlim(*time_window)
    if seen:
        axes[0].legend(handles=axes[0].get_legend_handles_labels()[0]+
                       [Patch(facecolor=c, label=n) for n, c in seen.items()],
                       loc='upper left', fontsize=8, ncol=4, framealpha=.9)
    fig.supxlabel('净空为控制器几何估计；色带为运动状态，非接口所有权或真实接触；Effort为发布前命令，非全程实际驱动力矩。', fontsize=9)
    fig.savefig(output_path, dpi=160)
    plt.close(fig)
    print(f'[INFO] 已保存单张{panels}联图: {output_path}')


def unused_output(path):
    """Preserve existing images, including the user's previous plot."""
    candidate = path
    number = 1
    while candidate.exists():
        candidate = path.with_name(f'{path.stem}_{number}{path.suffix}')
        number += 1
    return candidate


def main():
    directory = (Path(__file__).resolve().parents[4] /
                 'src/bbot_balance_controller/src/data_logs')
    parser = argparse.ArgumentParser(description=__doc__)
    default = directory / 'jump_velocity_control_log.csv'
    if not default.exists():
        default = directory / 'jump_control_log.csv'
    parser.add_argument('csvfile', nargs='?', type=Path, default=default)
    parser.add_argument('-o', '--output', type=Path, default=directory / 'jump_performance_single.png')
    parser.add_argument('--geometry', type=Path, help='同次运行的原生geometry.csv（轮世界位姿）')
    parser.add_argument('--landing-metrics', type=Path, help='既有验收metrics.json（真实first_touch_s及run）')
    parser.add_argument('--time-window', nargs=2, type=float, metavar=('START', 'END'),
                        help='相对日志首帧的显示窗口[s]，只改变视图，不筛选数据或重算验收')
    args = parser.parse_args()
    try:
        data = load_csv(args.csvfile)
        if bool(args.geometry) != bool(args.landing_metrics):
            raise ValueError('--geometry与--landing-metrics必须成对提供')
        native_axle = None
        if args.geometry:
            if args.geometry.resolve().parent != args.csvfile.resolve().parent:
                raise ValueError('控制CSV与geometry必须来自同一试次目录')
            native_axle = load_native_axle(data, args.geometry, args.landing_metrics)
        output = unused_output(args.output)
        if not output.parent.exists():
            raise ValueError(f'输出目录不存在: {output.parent}')
        plot_jump_performance(data, output, native_axle, args.time_window)
        print('[INFO] 未记录或无效量保留NaN；未运行仿真，未作物理达标判定。')
    except (OSError, ValueError, KeyError, TypeError) as exc:
        parser.exit(1, f'[ERROR] {exc}\n')


if __name__ == '__main__':
    main()
