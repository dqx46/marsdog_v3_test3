import csv
import numpy as np
import matplotlib.pyplot as plt

# 使用英文字符避免方块问题
LOG = "walk_log_20260628_153018.csv"

def read_motor_data(log_file, motor_id, mode_filter="trot_fwd"):
    times, targets, actuals = [], [], []
    with open(log_file) as f:
        reader = csv.DictReader(f)
        for r in reader:
            if r['motor_id'] == str(motor_id) and mode_filter in r['mode']:
                if r['target_deg'] != 'nan' and r['actual_deg'] != 'nan':
                    times.append(float(r['t_s']))
                    targets.append(float(r['target_deg']))
                    actuals.append(float(r['actual_deg']))
    return np.array(times), np.array(targets), np.array(actuals)

motors_to_plot = [
    (3, "FL Calf (Motor 3)"),
    (6, "FR Calf (Motor 6)"),
    (2, "FL Thigh Roll (Motor 2)"),
    (5, "FR Thigh Roll (Motor 5)")
]

plt.figure(figsize=(14, 12))

t_ref, _, _ = read_motor_data(LOG, 3)
if len(t_ref) > 0:
    t_start = t_ref[0] + 0.5
    
    for i, (mid, name) in enumerate(motors_to_plot):
        t, tgt, act = read_motor_data(LOG, mid)
        mask = (t >= t_start) & (t <= t_start + 2.0)
        t_m, tgt_m, act_m = t[mask], tgt[mask], act[mask]
        
        plt.subplot(len(motors_to_plot), 1, i+1)
        plt.title(f"{name} - Target vs Actual", fontsize=14)
        plt.plot(t_m, tgt_m, 'r--', linewidth=2, label="Target (Planned)")
        plt.plot(t_m, act_m, 'b-', linewidth=2, label="Actual (Motor)")
        
        err = act_m - tgt_m
        max_err_idx = np.argmax(np.abs(err))
        
        y_offset = 20 if err[max_err_idx] < 0 else -20
        plt.annotate(f'Max Error: {err[max_err_idx]:.1f} deg', 
                     xy=(t_m[max_err_idx], act_m[max_err_idx]), 
                     xytext=(t_m[max_err_idx]-0.15, act_m[max_err_idx] + y_offset),
                     arrowprops=dict(facecolor='black', shrink=0.05, width=1.5, headwidth=8),
                     fontsize=12, color='red', fontweight='bold')
        
        plt.ylabel("Angle (deg)", fontsize=12)
        plt.legend(loc='upper right', fontsize=12)
        plt.grid(True, linestyle=':', alpha=0.7)

    plt.xlabel("Time (s)", fontsize=12)
    plt.tight_layout()
    plt.savefig("tracking_all_joints_en.png", dpi=150)
    print("Plot saved to tracking_all_joints_en.png")
else:
    print("No data found.")
