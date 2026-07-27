import csv
import numpy as np
import matplotlib.pyplot as plt

LOGS = [
    ("1.00s (Slowest)", "walk_log_20260628_145553.csv"),
    ("0.75s (Sweet Spot)", "walk_log_20260628_163555.csv"),
    ("0.60s (Fast)", "walk_log_20260628_153803.csv"),
    ("0.40s (Extreme)", "walk_log_20260628_153018.csv")
]

def get_worst_window(log_file, motor_id=3, mode_filter="trot_fwd", window_size=2.0):
    times, targets, actuals = [], [], []
    try:
        with open(log_file) as f:
            reader = csv.DictReader(f)
            for r in reader:
                if r['motor_id'] == str(motor_id) and mode_filter in r['mode']:
                    if r['target_deg'] != 'nan' and r['actual_deg'] != 'nan':
                        times.append(float(r['t_s']))
                        targets.append(float(r['target_deg']))
                        actuals.append(float(r['actual_deg']))
    except Exception as e:
        return [], [], []
        
    times = np.array(times)
    targets = np.array(targets)
    actuals = np.array(actuals)
    
    if len(times) == 0:
        return [], [], []
        
    diffs = np.diff(times)
    split_indices = np.where(diffs < 0)[0] + 1
    
    t_segments = np.split(times, split_indices) if len(split_indices) > 0 else [times]
    tgt_segments = np.split(targets, split_indices) if len(split_indices) > 0 else [targets]
    act_segments = np.split(actuals, split_indices) if len(split_indices) > 0 else [actuals]
    
    max_err_global = 0
    worst_seg_idx = 0
    worst_err_idx_in_seg = 0
    
    for i, (tgt, act) in enumerate(zip(tgt_segments, act_segments)):
        if len(tgt) < 10: continue
        
        # 过滤掉电机卡死的数据 (连续10帧数值不变)
        act_diff = np.abs(np.diff(act))
        is_frozen = False
        for j in range(len(act_diff) - 10):
            if np.sum(act_diff[j:j+10]) < 0.1:
                is_frozen = True
                break
        
        # 如果这段数据包含卡死，我们只计算卡死之前的误差
        valid_len = len(tgt)
        if is_frozen:
            for j in range(len(act_diff) - 10):
                if np.sum(act_diff[j:j+10]) < 0.1:
                    valid_len = j
                    break
        
        if valid_len < 10: continue
        
        err = np.abs(act[:valid_len] - tgt[:valid_len])
        max_idx = np.argmax(err)
        if err[max_idx] > max_err_global:
            max_err_global = err[max_idx]
            worst_seg_idx = i
            worst_err_idx_in_seg = max_idx
            
    t_worst = t_segments[worst_seg_idx]
    tgt_worst = tgt_segments[worst_seg_idx]
    act_worst = act_segments[worst_seg_idx]
    
    center_t = t_worst[worst_err_idx_in_seg]
    t_start = max(t_worst[0], center_t - window_size/2)
    mask = (t_worst >= t_start) & (t_worst <= t_start + window_size)
    
    return t_worst[mask], tgt_worst[mask], act_worst[mask]

plt.figure(figsize=(14, 16))

for i, (title, log_file) in enumerate(LOGS):
    t_m, tgt_m, act_m = get_worst_window(log_file, 3)
    if len(t_m) > 0:
        t_m = t_m - t_m[0] # Normalize time to start at 0
        
        plt.subplot(4, 1, i+1)
        plt.title(f"Period: {title} - FL Calf (Motor 3)", fontsize=14)
        plt.plot(t_m, tgt_m, 'r--', linewidth=2, label="Target (Planned)")
        plt.plot(t_m, act_m, 'b-', linewidth=2, label="Actual (Motor)")
        
        err = act_m - tgt_m
        max_err_idx = np.argmax(np.abs(err))
        
        y_offset = 25 if err[max_err_idx] < 0 else -25
        color = 'red' if abs(err[max_err_idx]) > 15 else ('orange' if abs(err[max_err_idx]) > 5 else 'green')
        
        plt.annotate(f'Max Error: {err[max_err_idx]:.1f} deg', 
                     xy=(t_m[max_err_idx], act_m[max_err_idx]), 
                     xytext=(t_m[max_err_idx]-0.15, act_m[max_err_idx] + y_offset),
                     arrowprops=dict(facecolor='black', shrink=0.05, width=1.5, headwidth=8),
                     fontsize=13, color=color, fontweight='bold')
                     
        plt.ylabel("Angle (deg)", fontsize=12)
        plt.legend(loc='upper right', fontsize=12)
        plt.grid(True, linestyle=':', alpha=0.7)

plt.xlabel("Time (s)", fontsize=12)
plt.tight_layout()
plt.savefig("tracking_all_periods_fixed.png", dpi=150)
print("Plot saved to tracking_all_periods_fixed.png")
