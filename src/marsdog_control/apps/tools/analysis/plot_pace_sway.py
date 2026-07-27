import csv
import numpy as np
import matplotlib.pyplot as plt

LOG = "walk_log_20260629_111923.csv"

times, roll, sway, react = [], [], [], []
with open(LOG) as f:
    reader = csv.DictReader(f)
    for r in reader:
        if r['mode'] == 'pace_fwd' and r['motor_id'] == '3':
            if r['imu_roll_deg'] != 'nan':
                times.append(float(r['t_s']))
                roll.append(float(r['imu_roll_deg']))
                sway.append(float(r['lateral_sway_mm']))
                react.append(float(r['reactive_deg']))

times = np.array(times)
roll = np.array(roll)
sway = np.array(sway)
react = np.array(react)

t = times - times[0]

fig, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
mask = (t > 2.0) & (t < 6.0)

axs[0].plot(t[mask], roll[mask], 'g-', label="Roll (deg)")
axs[0].legend()
axs[0].grid()

axs[1].plot(t[mask], sway[mask], 'm-', label="Sway (mm)")
axs[1].legend()
axs[1].grid()

axs[2].plot(t[mask], react[mask], 'c-', label="Reactive (deg)")
axs[2].legend()
axs[2].grid()

plt.tight_layout()
plt.savefig("pace_sway_check.png")
