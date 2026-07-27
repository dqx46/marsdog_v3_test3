import csv
import numpy as np
import matplotlib.pyplot as plt

LOG_FAST = "walk_log_20260628_153018.csv" # 0.4s period (Worst tracking)
LOG_SLOW = "walk_log_20260628_145553.csv" # 1.0s period (Best tracking)

def read_motor_segment(log_file, motor_id, mode_filter="trot_fwd"):
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
        return times, targets, actuals
        
    diffs = np.diff(times)
    split_indices = np.where(diffs < 0)[0] + 1
    
    if len(split_indices) > 0:
        t_segments = np.split(times, split_indices)
        tgt_segments = np.split(targets, split_indices)
        act_segments = np.split(actuals, split_indices)
        
        longest_idx = np.argmax([len(seg) for seg in t_segments])
        return t_segments[longest_idx], tgt_segments[longest_idx], act_segments[longest_idx]
    
    return times, targets, actuals

plt.figure(figsize=(12, 10))

# --- Plot 1: SLOW (Best Tracking) ---
t_s, tgt_s, act_s = read_motor_segment(LOG_SLOW, 3)
if len(t_s) > 0:
    # Take a 2-second window
    t_start = t_s[0] + 0.5
    mask = (t_s >= t_start) & (t_s <= t_start + 2.0)
    t_m, tgt_m, act_m = t_s[mask], tgt_s[mask], act_s[mask]
    
    # Normalize time to start at 0 for comparison
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
t_f, tgt_f, act_f = read_motor_segment(LOG_FAST, 3)
if len(t_f) > 0:
    # Take a 2-second window
    t_start = t_f[0] + 0.5
    mask = (t_f >= t_start) & (t_f <= t_start + 2.0)
    t_m, tgt_m, act_m = t_f[mask], tgt_f[mask], act_f[mask]
    
    # Normalize time to start at 0
    t_m = t_m - t_m[0]
    
    plt.subplot(2, 1, 2)
    plt.title("WORST Tracking: Fast Speed (0.4s Period) - FL Calf", fontsize=14)
    plt.plot(t_m, tgt_m, 'r--', linewidth=2, label="Target (Planned)")
    plt.plot(t_m, act_m, 'b-', linewidth=2, label="Actual (Motor)")
    
    err = act_m - tgt_m
    max_err_idx = np.argmax(np.abs(err))
    plt.annotate(f'Max Error: {err[max_err_idx]:.1f} deg\n(Motor Torque Saturated)', 
                 xy=(t_m[max_err_idx], act_m[max_err_idx]), 
                 xytext=(t_m[max_err_idx]-0.15, act_m[max_err_idx] - 25),
                 arrowprops=dict(facecolor='black', shrink=0.05, width=1.5, headwidth=8),
                 fontsize=12, color='red', fontweight='bold')
                 
    plt.xlabel("Time (s)", fontsize=12)
    plt.ylabel("Angle (deg)", fontsize=12)
    plt.legend(loc='upper right', fontsize=12)
    plt.grid(True, linestyle=':', alpha=0.7)

plt.tight_layout()
plt.savefig("tracking_comparison.png", dpi=150)
print("Plot saved to tracking_comparison.png")
