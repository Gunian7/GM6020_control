"""
GM6020 阶跃响应数据分析工具
===========================================
用法：
  1. 将串口输出的 CSV 数据保存到文件（去掉以 # 开头的注释行）
  2. python3 analyze_step_response.py <数据文件.csv>

如果没有实际数据，运行本脚本会生成示例数据用于演示分析流程。
"""

import sys
import csv
import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.ticker import MaxNLocator

# ======================== 辅助函数 ========================

def find_settling_time(t, actual, target, tol_percent=2):
    """
    找到进入 ±tol_percent% 稳态误差带后不再离开的时间点。
    返回 (settling_time_ms, settling_index)
    """
    tol = abs(target) * tol_percent / 100.0
    if tol < 1e-6:
        tol = 1.0  # 目标为0时用绝对容差
    
    n = len(actual)
    settle_idx = n - 1
    
    for i in range(n - 1, -1, -1):
        if abs(actual[i] - target) > tol:
            settle_idx = i
            break
    
    # 取稳态带内的第一个点
    for i in range(settle_idx, n):
        if abs(actual[i] - target) <= tol:
            return t[i], i
    
    return t[-1], n - 1


def analyze_step_segment(t, actual, target, step_time, step_idx, label=""):
    """分析单个阶跃响应的性能指标"""
    
    # 截取阶跃后的数据（阶跃后 3000ms）
    start = step_idx
    end = min(step_idx + 3000, len(t))
    
    if end - start < 5:
        return None
    
    t_seg = [ti - step_time for ti in t[start:end]]  # 相对时间 (ms)
    act_seg = actual[start:end]
    
    # 1. 超调量 Overshoot
    if target > 0:
        peak = max(act_seg)
        overshoot = (peak - target) / target * 100.0
    else:
        peak = min(act_seg)
        overshoot = (peak - target) / abs(target) * 100.0
    
    peak_idx = np.argmax(act_seg) if target > 0 else np.argmin(act_seg)
    peak_time = t_seg[peak_idx]
    
    # 2. 上升时间 (10% ~ 90%)
    target_10 = step_value + 0.1 * (target - step_value)
    target_90 = step_value + 0.9 * (target - step_value)
    
    t10 = None
    t90 = None
    for i, (tv, av, sv) in enumerate(zip(t_seg, act_seg, [step_value]*len(t_seg))):
        if target > step_value:  # 上升
            if t10 is None and av >= target_10:
                t10 = tv
            if t90 is None and av >= target_90:
                t90 = tv
                break
        else:  # 下降
            if t10 is None and av <= target_10:
                t10 = tv
            if t90 is None and av <= target_90:
                t90 = tv
                break
    
    rise_time = (t90 - t10) if (t10 is not None and t90 is not None) else None
    
    # 3. 调节时间 (进入 ±2% 稳态带)
    settle_time, settle_idx_local = find_settling_time(t_seg, act_seg, target, 2)
    
    # 4. 稳态误差（取最后 500ms 的平均）
    steady_data = act_seg[-min(500, len(act_seg)):]
    steady_error = abs(np.mean(steady_data) - target)
    
    print(f"  [{label}]")
    print(f"    阶跃: {step_value:.0f}° → {target:.0f}°")
    print(f"    峰值: {peak:.2f}° (@ {peak_time:.0f}ms)")
    print(f"    超调量: {overshoot:.2f}%")
    print(f"    上升时间(10-90%): {rise_time:.1f}ms" if rise_time else "    上升时间: N/A")
    print(f"    峰值时间: {peak_time:.0f}ms")
    print(f"    调节时间(±2%): {settle_time:.0f}ms")
    print(f"    稳态误差: {steady_error:.2f}°")
    print()
    
    return {
        'seg_t': t_seg,
        'seg_act': act_seg,
        'target': target,
        'overshoot': overshoot,
        'peak_time': peak_time,
        'rise_time': rise_time,
        'settle_time': settle_time,
        'steady_error': steady_error,
        'step_time': step_time,
    }


# ======================== 数据加载 ========================

def load_data(filepath):
    """从 CSV 文件加载数据"""
    t = []
    target = []
    actual = []
    speed = []
    phase = []
    
    with open(filepath, 'r') as f:
        reader = csv.reader(f)
        for row in reader:
            if len(row) < 4:
                continue
            try:
                t.append(float(row[0]))
                target.append(float(row[1]))
                actual.append(float(row[2]))
                speed.append(float(row[3]))
                if len(row) >= 5:
                    phase.append(int(row[4]))
                else:
                    phase.append(0)
            except (ValueError, IndexError):
                continue
    
    return np.array(t), np.array(target), np.array(actual), np.array(speed), np.array(phase)


# ======================== 绘图 ========================

def plot_results(t, target, actual, speed, segments):
    """绘制完整的响应曲线"""
    
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(14, 8), sharex=True)
    
    # ---- 上: 位置响应 ----
    ax1.plot(t, target, 'r--', linewidth=1.5, label='Target Angle', alpha=0.7)
    ax1.plot(t, actual, 'b-', linewidth=1.0, label='Actual Angle')
    
    # 标注阶跃事件
    for seg in segments:
        if seg:
            ax1.axvline(x=seg['step_time'], color='gray', linestyle=':', alpha=0.5)
            ax1.annotate(f'{seg["target"]:.0f}°',
                        xy=(seg['step_time'], seg['target']),
                        xytext=(seg['step_time'] + 100, seg['target'] + 15),
                        fontsize=9, color='darkred',
                        arrowprops=dict(arrowstyle='->', color='gray', alpha=0.6))
    
    ax1.set_ylabel('Angle (deg)')
    ax1.set_title('GM6020 Step Response - Position')
    ax1.legend(loc='best')
    ax1.grid(True, alpha=0.3)
    
    # ---- 下: 速度响应 ----
    ax2.plot(t, speed, 'g-', linewidth=0.8, label='Actual Speed (rpm)')
    ax2.set_xlabel('Time (ms)')
    ax2.set_ylabel('Speed (rpm)')
    ax2.set_title('GM6020 Step Response - Speed')
    ax2.legend(loc='best')
    ax2.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('gm6020_step_response.png', dpi=150)
    print(f"\n图像已保存: gm6020_step_response.png")
    plt.show()


def plot_single_step(seg, title=""):
    """绘制单个阶跃的放大图"""
    if seg is None:
        return
    
    fig, ax = plt.subplots(figsize=(10, 5))
    
    t = seg['seg_t']
    act = seg['seg_act']
    target = seg['target']
    
    ax.plot(t, act, 'b-', linewidth=1.5, label='Actual')
    ax.axhline(y=target, color='r', linestyle='--', linewidth=1, label=f'Target ({target}°)')
    
    # ±2% 稳态带
    tol = abs(target) * 0.02 if abs(target) > 1 else 1.0
    ax.axhline(y=target + tol, color='orange', linestyle=':', linewidth=0.8, alpha=0.6)
    ax.axhline(y=target - tol, color='orange', linestyle=':', linewidth=0.8, alpha=0.6,
               label=f'±2% band (±{tol:.1f}°)')
    
    # 标注性能指标
    if seg['rise_time']:
        ax.annotate(f'Rise: {seg["rise_time"]:.0f}ms',
                    xy=(200, target * 0.3), fontsize=10,
                    bbox=dict(boxstyle='round,pad=0.3', facecolor='lightblue', alpha=0.5))
    
    ax.annotate(f'Overshoot: {seg["overshoot"]:.1f}%',
                xy=(seg['peak_time'], max(act)),
                fontsize=10, color='darkred',
                bbox=dict(boxstyle='round,pad=0.3', facecolor='mistyrose', alpha=0.8))
    
    ax.annotate(f'Settle: {seg["settle_time"]:.0f}ms',
                xy=(seg['settle_time'], target),
                fontsize=10,
                bbox=dict(boxstyle='round,pad=0.3', facecolor='lightgreen', alpha=0.5))
    
    ax.set_xlabel('Time (ms)')
    ax.set_ylabel('Angle (deg)')
    ax.set_title(title or f'Step Response: {seg["target"]:.0f}°')
    ax.legend(loc='best')
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    safe_name = f'step_{seg["target"]:.0f}deg_detailed.png'.replace('-', 'neg')
    plt.savefig(safe_name, dpi=150)
    print(f"详细图已保存: {safe_name}")
    plt.show()


# ======================== 生成示例数据 ========================

def generate_sample_data():
    """生成模拟的阶跃响应数据用于演示分析流程"""
    print("=" * 60)
    print("  未找到实际数据文件。生成示例数据用于演示。")
    print("=" * 60)
    print()
    
    np.random.seed(42)
    dt = 0.005  # 5ms 采样间隔
    total_time = 40.0  # 40s
    t = np.arange(0, total_time, dt)
    
    target = np.zeros_like(t)
    actual = np.zeros_like(t)
    speed = np.zeros_like(t)
    phase = np.zeros_like(t, dtype=int)
    
    # 模拟二阶系统响应
    def step_response(t_arr, amp, t0, wn=8.0, zeta=0.6):
        """模拟二阶阶跃响应: overshoot ≈ exp(-πζ/√(1-ζ²))"""
        resp = np.zeros_like(t_arr)
        for i, tt in enumerate(t_arr):
            if tt < t0:
                resp[i] = 0
            else:
                td = tt - t0
                if td < 0.01:
                    resp[i] = 0
                else:
                    # 二阶系统阶跃响应
                    wd = wn * math.sqrt(1 - zeta * zeta)
                    phi = math.acos(zeta)
                    resp[i] = amp * (1 - math.exp(-zeta * wn * td) * 
                                     math.sin(wd * td + phi) / math.sin(phi))
        return resp
    
    # 构造测试序列
    phases_def = [
        (0, 0, 0, 0),       # 归零
        (5, 90, 8.0, 0.6),  # +90° 阶跃
        (10, 90, 0, 0),     # 保持
        (15, -90, 8.0, 0.6),# -90° 阶跃
        (20, -90, 0, 0),    # 保持
        (25, 0, 8.0, 0.6),  # 归零
        (30, 180, 6.5, 0.55),# +180° 较大阶跃（阻尼略低，超调略大）
        (35, 180, 0, 0),    # 保持
    ]
    
    prev_target = 0
    for start_time, amp, wn, zeta in phases_def:
        mask = (t >= start_time) & (t < start_time + 5)
        target[mask] = amp
        if amp != prev_target or True:
            step_resp = step_response(t, amp - prev_target, start_time, wn, zeta)
            actual += step_resp
        prev_target = amp
        phase[mask] = int(phases_def.index((start_time, amp, wn, zeta)) 
                          if (start_time, amp, wn, zeta) in phases_def else 0)
    
    # 添加噪声
    actual += np.random.normal(0, 0.3, len(t))
    speed = np.gradient(actual, dt) / 360 * 60  # deg/s → rpm（粗略模拟）
    speed += np.random.normal(0, 5, len(t))
    
    return t * 1000, target, actual, speed, phase


# ======================== 主程序 ========================

if __name__ == '__main__':
    # 默认找同目录的 step_response_data.csv
    filepath = sys.argv[1] if len(sys.argv) > 1 else "step_response_data.csv"
    
    try:
        t, target, actual, speed, phase = load_data(filepath)
        print(f"已加载数据: {len(t)} 条记录")
        print(f"时间范围: {t[0]:.0f} ~ {t[-1]:.0f} ms ({t[-1]/1000:.1f}s)")
    except (FileNotFoundError, Exception) as e:
        print(f"数据文件 '{filepath}' 无法加载: {e}")
        t, target, actual, speed, phase = generate_sample_data()
    
    global step_value
    step_value = target[0]
    
    # ---- 寻找阶跃事件 ----
    step_events = []
    for i in range(1, len(target)):
        if abs(target[i] - target[i-1]) > 10:  # 角度差 > 10° 认为是阶跃
            step_events.append((i, t[i], target[i-1], target[i]))
    
    # ---- 分析每个阶跃 ----
    segments = []
    print("\n" + "=" * 60)
    print("  阶跃响应性能分析")
    print("=" * 60 + "\n")
    
    for idx, step_idx_global, step_time, step_value, step_target in step_events:
        seg = analyze_step_segment(
            t, actual, step_target,
            step_time, step_idx_global,
            label=f"Step {idx+1}"
        )
        segments.append(seg)
    
    # ---- 绘总图 ----
    plot_results(t, target, actual, speed, segments)
    
    # ---- 绘单个阶跃放大图 ----
    for i, seg in enumerate(segments):
        if seg:
            plot_single_step(seg, title=f"Step {i+1}: {seg['target']:.0f}° Detail")
    
    print("\n" + "=" * 60)
    print("  分析完成")
    print("=" * 60)
    print("在图像中查看:")
    print("  超调量 = (峰值 - 目标值) / 目标值 × 100%")
    print("  调节时间 = 输出进入 ±2% 稳态带后不再离开的时间")
    print("  上升时间 = 10% ~ 90% 目标值的时间")
    print("  稳态误差 = 稳定后平均值与目标的偏差")
