import csv
import numpy as np
import matplotlib.pyplot as plt
import math

LOG = "walk_log_20260629_111923.csv"

times, roll, pitch, p_mm, d_mm, react, sway, act, tgt = [], [], [], [], [], [], [], [], []
with open(LOG) as f:
    reader = csv.DictReader(f)
    for r in reader:
        if r['mode'] == 'pace_fwd' and r['motor_id'] == '3':
            if r['imu_roll_deg'] != 'nan':
                times.append(float(r['t_s']))
                roll.append(float(r['imu_roll_deg']))
                pitch.append(float(r['imu_pitch_deg']))
                p_mm.append(float(r['imu_roll_p_mm']))
                d_mm.append(float(r['imu_roll_d_mm']))
                react.append(float(r['reactive_deg']))
                sway.append(float(r['lateral_sway_mm']))
                act.append(float(r['actual_deg']))
                tgt.append(float(r['target_deg']))

if not times:
    print("No pace_fwd data found.")
    exit()

times = np.array(times)
roll = np.array(roll)
pitch = np.array(pitch)
p_mm = np.array(p_mm)
d_mm = np.array(d_mm)
react = np.array(react)
sway = np.array(sway)
act = np.array(act)
tgt = np.array(tgt)

# Find longest segment
diffs = np.diff(times)
split_indices = np.where(diffs < 0)[0] + 1
longest_idx = np.argmax([len(s) for s in np.split(times, split_indices)])

t = np.split(times, split_indices)[longest_idx]
r = np.split(roll, split_indices)[longest_idx]
p = np.split(pitch, split_indices)[longest_idx]
p_val = np.split(p_mm, split_indices)[longest_idx]
d_val = np.split(d_mm, split_indices)[longest_idx]
react_val = np.split(react, split_indices)[longest_idx]
s_val = np.split(sway, split_indices)[longest_idx]
a_val = np.split(act, split_indices)[longest_idx]
tg_val = np.split(tgt, split_indices)[longest_idx]

t = t - t[0]

print(f"--- Pace Analysis ---")
print(f"Duration: {t[-1]:.2f}s")
print(f"Roll Range: {np.min(r):.1f}° to {np.max(r):.1f}° (Amp: {np.max(r)-np.min(r):.1f}°)")
print(f"Pitch Range: {np.min(p):.1f}° to {np.max(p):.1f}° (Amp: {np.max(p)-np.min(p):.1f}°)")
print(f"Max P Comp: {np.max(np.abs(p_val)):.1f} mm")
print(f"Max D Comp: {np.max(np.abs(d_val)):.1f} mm")
print(f"Max Reactive: {np.max(np.abs(react_val)):.2f}°")
print(f"Max Tracking Error (Motor 3): {np.max(np.abs(a_val - tg_val)):.1f}°")

# Let's see if the expected roll is correctly calculated
period = 0.75
stance_ratio = 0.6
lateral_sway = 0.040
body_height = 0.24
delay = 0.18

expected_roll = []
for time_val in t:
    t_delayed = max(0.0, time_val - delay)
    phase = (t_delayed / period) % 1.0
    if phase < stance_ratio:
        t_norm = phase / stance_ratio
        lat_offset = lateral_sway * math.sin(math.pi * t_norm)
    else:
        t_norm = (phase - stance_ratio) / (1.0 - stance_ratio)
        lat_offset = -lateral_sway * math.sin(math.pi * t_norm)
    expected_roll.append(math.degrees(lat_offset / body_height))

expected_roll = np.array(expected_roll)
eff_roll = r - expected_roll

fig, axs = plt.subplots(4, 1, figsize=(12, 12), sharex=True)

mask = t > 0.0

axs[0].set_title("Roll Decoupling Check (Pace)", fontsize=14)
axs[0].plot(t[mask], r[mask], 'g-', label="Raw IMU Roll")
axs[0].plot(t[mask], expected_roll[mask], 'm--', label="Expected Roll (from Sway)")
axs[0].plot(t[mask], eff_roll[mask], 'b-', linewidth=2, label="Effective Roll (Fed to IMU Ctrl)")
axs[0].legend()
axs[0].grid(True)

axs[1].set_title("IMU Compensation Outputs", fontsize=12)
axs[1].plot(t[mask], p_val[mask], 'b-', label="P-Term")
axs[1].plot(t[mask], d_val[mask], 'r-', label="D-Term")
axs[1].legend()
axs[1].grid(True)

axs[2].set_title("Raibert Reactive Stepping", fontsize=12)
axs[2].plot(t[mask], react_val[mask], 'c-', label="Reactive Deg")
axs[2].legend()
axs[2].grid(True)

axs[3].set_title("Motor 3 Tracking", fontsize=12)
axs[3].plot(t[mask], tg_val[mask], 'r--', label="Target")
axs[3].plot(t[mask], a_val[mask], 'b-', label="Actual")
axs[3].legend()
axs[3].grid(True)

plt.tight_layout()
plt.savefig("pace_analysis.png")
print("Plot saved to pace_analysis.png")
