import csv
import numpy as np
import matplotlib.pyplot as plt
import math

LOG = "walk_log_20260628_173421.csv"

times, roll, pitch, p_mm, d_mm, react = [], [], [], [], [], []
with open(LOG) as f:
    reader = csv.DictReader(f)
    for r in reader:
        if 'trot_fwd' in r['mode'] and r['motor_id'] == '3':
            if r['imu_roll_deg'] != 'nan':
                times.append(float(r['t_s']))
                roll.append(float(r['imu_roll_deg']))
                pitch.append(float(r['imu_pitch_deg']))
                p_mm.append(float(r['imu_roll_p_mm']))
                d_mm.append(float(r['imu_roll_d_mm']))
                react.append(float(r['reactive_deg']))

times = np.array(times)
roll = np.array(roll)
pitch = np.array(pitch)
p_mm = np.array(p_mm)
d_mm = np.array(d_mm)
react = np.array(react)

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

t = t - t[0]

print(f"--- Stability Analysis ---")
print(f"Duration: {t[-1]:.2f}s")
print(f"Roll Range: {np.min(r):.1f}° to {np.max(r):.1f}° (Amp: {np.max(r)-np.min(r):.1f}°)")
print(f"Pitch Range: {np.min(p):.1f}° to {np.max(p):.1f}° (Amp: {np.max(p)-np.min(p):.1f}°)")
print(f"Max P Comp: {np.max(np.abs(p_val)):.1f} mm")
print(f"Max D Comp: {np.max(np.abs(d_val)):.1f} mm")
print(f"Max Reactive: {np.max(np.abs(react_val)):.2f}°")

# Reconstruct expected roll to check sign
period = 0.75
stance_ratio = 0.6
lateral_sway = 0.015
body_height = 0.24

expected_roll = []
for time_val in t:
    phase = (time_val / period) % 1.0
    if phase < stance_ratio:
        t_norm = phase / stance_ratio
        lat_offset = lateral_sway * math.sin(math.pi * t_norm)
    else:
        t_norm = (phase - stance_ratio) / (1.0 - stance_ratio)
        lat_offset = -lateral_sway * math.sin(math.pi * t_norm)
    expected_roll.append(math.degrees(lat_offset / body_height))

expected_roll = np.array(expected_roll)
eff_roll = r - expected_roll

fig, axs = plt.subplots(3, 1, figsize=(12, 10), sharex=True)

mask = (t > 2.0) & (t < 6.0)

axs[0].set_title("Roll Decoupling Check", fontsize=14)
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

plt.tight_layout()
plt.savefig("stability_check.png")
print("Plot saved to stability_check.png")
