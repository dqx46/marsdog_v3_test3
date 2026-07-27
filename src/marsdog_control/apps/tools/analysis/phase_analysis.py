import csv
import numpy as np
import matplotlib.pyplot as plt

LOG = "walk_log_20260628_172304.csv"

times, roll, pitch = [], [], []
with open(LOG) as f:
    reader = csv.DictReader(f)
    for r in reader:
        if 'trot_fwd' in r['mode'] and r['motor_id'] == '3':
            if r['imu_roll_deg'] != 'nan':
                times.append(float(r['t_s']))
                roll.append(float(r['imu_roll_deg']))
                pitch.append(float(r['imu_pitch_deg']))

times = np.array(times)
roll = np.array(roll)
pitch = np.array(pitch)

period = 0.75
phases = (times / period) % 1.0

plt.figure(figsize=(10, 6))
plt.scatter(phases, roll, c='g', alpha=0.5, label='Roll')
plt.scatter(phases, pitch, c='m', alpha=0.5, label='Pitch')
plt.axvline(0.5, color='k', linestyle='--')
plt.xlabel("Gait Phase (0 to 1)")
plt.ylabel("Angle (deg)")
plt.legend()
plt.title("Roll/Pitch vs Gait Phase")
plt.savefig("phase_analysis.png")
