import csv
import numpy as np
import matplotlib.pyplot as plt

LOG_FAST = "walk_log_20260628_153018.csv" # 0.4s period
LOG_SLOW = "walk_log_20260628_145553.csv" # 1.0s period

def get_worst_window(log_file, motor_id, mode_filter="trot_fwd", window_size=2.0):
    times, targets, actuals = [], [], []
    with open(log_file) as f:
        reader = csv.DictReader(f)
        for r in reader:
            if r['motor_id'] == str(motor_id) and mode_filter in r['mode']:
                if r['target_deg'] != 'nan' and r['actual_deg'] != 'nan':
                    times.append(float(r['t_s']))
                    targets.append(float(r['target_deg']))
                    actuals.append(float(r['actual_deg']))
    
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
    
    # 找到包含绝对最大误差的那个段落
    max_err_global = 0
    worst_seg_idx = 0
    worst_err_idx_in_seg = 0
    
    for i, (tgt, act) in enumerate(zip(tgt_segments, act_segments)):
        if len(tgt) == 0: continue
        err = np.abs(act - tgt)
        max_idx = np.argmax(err)
        if err[max_idx] > max_err_global:
            max_err_global = err[max_idx]
            worst_seg_idx = i
            worst_err_idx_in_seg = max_idx
            
    t_worst = t_segments[worst_seg_idx]
    tgt_worst = tgt_segments[worst_seg_idx]
    act_worst = act_segments[worst_seg_idx]
    
    # 截取包含最大误差的窗口
    center_t = t_worst[worst_err_idx_in_seg]
    t_start = max(t_worst[0], center_t - window_size/2)
    mask = (t_worst >= t_start) & (t_worst <= t_start + window_size)
    
    return t_worst[mask], tgt_worst[mask], act_worst[mask]

plt.figure(figsize=(12, 10))

# --- Plot 1: SLOW (Best Tracking) ---
t_m, tgt_m, act_m = get_worst_window(LOG_SLOW, 3)
if len(t_m) > 0:
    t_m = t_m - t_m[0]
    plt.subplot(2, 1, 1)
    plt.title("BEST Tracking: Slow Speed (1.0s Period) - FL Calf", fontsize=14)
    plt.plot(t_m, tgt_m, 'r--', linewidth=2, label="Target (Planned)")
    plt.plot(t_m, act_m, 'b-', linewidth=2, label="Actual (Motor)")
    
    err = act_m - tgt_m
    max_err_idx = np.argmax(np.abs(err))
    plt.annotate(f'Max Error: {err[max_err_idx]:.1f} deg', 
                 xy=(t_m[max_err_idx], act_m[max_err_idx]), 
                 xytext=(t_m[max_err_idx]-0.1, act_m[max_err_idx] + 15),
                 arrowprops=dict(facecolor='black', shrink=0.05, width=1.5, headwidth=8),
                 fontsize=12, color='green', fontweight='bold')
                 
    plt.ylabel("Angle (deg)", fontsize=12)
    plt.legend(loc='upper right', fontsize=12)
    plt.grid(True, linestyle=':', alpha=0.7)

# --- Plot 2: FAST (Worst Tracking) ---
t_m, tgt_m, act_m = get_worst_window(LOG_FAST, 3)
if len(t_m) > 0:
    t_m = t_m - t_m[0]
    plt.subplot(2, 1, 2)
    plt.title("WORST Tracking: Fast Speed (0.4s Period) - FL Calf", fontsize=14)
    plt.plot(t_m, tgt_m, 'r--', linewidth=2, label="Target (Planned)")
    plt.plot(t_m, act_m, 'b-', linewidth=2, label="Actual (Motor)")
    
    err = act_m - tgt_m
    max_err_idx = np.argmax(np.abs(err))
    plt.annotate(f'Max Error: {err[max_err_idx]:.1f} deg\n(Motor Torque Saturated)', 
                 xy=(t_m[max_err_idx], act_m[max_err_idx]), 
                 xytext=(t_m[max_err_idx]-0.25, act_m[max_err_idx] - 25),
                 arrowprops=dict(facecolor='black', shrink=0.05, width=1.5, headwidth=8),
                 fontsize=12, color='red', fontweight='bold')
                 
    plt.xlabel("Time (s)", fontsize=12)
    plt.ylabel("Angle (deg)", fontsize=12)
    plt.legend(loc='upper right', fontsize=12)
    plt.grid(True, linestyle=':', alpha=0.7)

plt.tight_layout()
plt.savefig("tracking_comparison_fixed.png", dpi=150)
print("Plot saved to tracking_comparison_fixed.png")
