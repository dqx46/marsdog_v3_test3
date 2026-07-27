import csv
import numpy as np
import matplotlib.pyplot as plt

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

# Read fl_calf (Motor 3) - Worst tracking
t_calf, tgt_calf, act_calf = read_motor_data(LOG, 3)
# Read fl_thigh_roll (Motor 2)
t_thigh, tgt_thigh, act_thigh = read_motor_data(LOG, 2)

# Select a 2-second window for clarity (e.g., from t=2.0 to t=4.0 of the trot segment)
if len(t_calf) > 0:
    t_start = t_calf[0] + 0.5
    mask = (t_calf >= t_start) & (t_calf <= t_start + 1.5)
    t_c, tgt_c, act_c = t_calf[mask], tgt_calf[mask], act_calf[mask]
    
    mask_t = (t_thigh >= t_start) & (t_thigh <= t_start + 1.5)
    t_t, tgt_t, act_t = t_thigh[mask_t], tgt_thigh[mask_t], act_thigh[mask_t]

    # Calculate velocities (deg/s)
    dt_c = np.diff(t_c)
    vel_tgt_c = np.diff(tgt_c) / dt_c
    vel_act_c = np.diff(act_c) / dt_c
    
    dt_t = np.diff(t_t)
    vel_tgt_t = np.diff(tgt_t) / dt_t
    vel_act_t = np.diff(act_t) / dt_t

    plt.figure(figsize=(12, 10))
    
    # Plot Calf Position
    plt.subplot(4, 1, 1)
    plt.title("fl_calf (Motor 3) - Position Tracking (0.4s Period)")
    plt.plot(t_c, tgt_c, 'r--', label="Target (Planned)")
    plt.plot(t_c, act_c, 'b-', label="Actual")
    plt.ylabel("Angle (deg)")
    plt.legend()
    plt.grid(True)

    # Plot Calf Velocity
    plt.subplot(4, 1, 2)
    plt.title("fl_calf (Motor 3) - Velocity")
    plt.plot(t_c[:-1], vel_tgt_c, 'r--', label="Target Vel")
    plt.plot(t_c[:-1], vel_act_c, 'b-', label="Actual Vel")
    plt.ylabel("Speed (deg/s)")
    plt.legend()
    plt.grid(True)
    
    # Plot Thigh Roll Position
    plt.subplot(4, 1, 3)
    plt.title("fl_thigh_roll (Motor 2) - Position Tracking")
    plt.plot(t_t, tgt_t, 'r--', label="Target (Planned)")
    plt.plot(t_t, act_t, 'b-', label="Actual")
    plt.ylabel("Angle (deg)")
    plt.legend()
    plt.grid(True)
    
    # Plot Thigh Roll Velocity
    plt.subplot(4, 1, 4)
    plt.title("fl_thigh_roll (Motor 2) - Velocity")
    plt.plot(t_t[:-1], vel_tgt_t, 'r--', label="Target Vel")
    plt.plot(t_t[:-1], vel_act_t, 'b-', label="Actual Vel")
    plt.xlabel("Time (s)")
    plt.ylabel("Speed (deg/s)")
    plt.legend()
    plt.grid(True)

    plt.tight_layout()
    plt.savefig("tracking_analysis.png", dpi=150)
    print("Plot saved to tracking_analysis.png")
else:
    print("No data found.")
