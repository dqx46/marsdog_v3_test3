import csv
import numpy as np
import matplotlib.pyplot as plt

LOGS = [
    ("1.00s (Stable but Bouncy)", "walk_log_20260628_145553.csv"),
    ("0.75s (Sways & Falls)", "walk_log_20260628_163555.csv"),
    ("0.60s (Violent Sway/Fall)", "walk_log_20260628_153631.csv"), # Use the longer 0.6s log
    ("0.40s (Torque Saturation)", "walk_log_20260628_153018.csv")
]

CALF_MOTORS = ['3', '6', '9', '12']

def get_valid_data(log_file):
    # We will extract the longest valid segment for the worst calf motor
    # and also extract IMU roll/pitch for that segment
    
    best_seg_t = []
    best_seg_tgt = []
    best_seg_act = []
    best_seg_roll = []
    best_seg_pitch = []
    best_mid = '3'
    max_err_global = -1
    
    for mid in CALF_MOTORS:
        times, targets, actuals, rolls, pitches = [], [], [], [], []
        try:
            with open(log_file) as f:
                reader = csv.DictReader(f)
                for r in reader:
                    if 'trot_fwd' in r['mode'] and r['motor_id'] == mid:
                        if r['target_deg'] != 'nan' and r['actual_deg'] != 'nan':
                            times.append(float(r['t_s']))
                            targets.append(float(r['target_deg']))
                            actuals.append(float(r['actual_deg']))
                            rolls.append(float(r['imu_roll_deg']) if r['imu_roll_deg'] != 'nan' else 0)
                            pitches.append(float(r['imu_pitch_deg']) if r['imu_pitch_deg'] != 'nan' else 0)
        except Exception as e:
            continue
            
        if len(times) == 0: continue
        
        times = np.array(times)
        targets = np.array(targets)
        actuals = np.array(actuals)
        rolls = np.array(rolls)
        pitches = np.array(pitches)
        
        diffs = np.diff(times)
        split_indices = np.where(diffs < 0)[0] + 1
        
        t_segs = np.split(times, split_indices) if len(split_indices) > 0 else [times]
        tgt_segs = np.split(targets, split_indices) if len(split_indices) > 0 else [targets]
        act_segs = np.split(actuals, split_indices) if len(split_indices) > 0 else [actuals]
        roll_segs = np.split(rolls, split_indices) if len(split_indices) > 0 else [rolls]
        pitch_segs = np.split(pitches, split_indices) if len(split_indices) > 0 else [pitches]
        
        for i in range(len(t_segs)):
            t, tgt, act, r, p = t_segs[i], tgt_segs[i], act_segs[i], roll_segs[i], pitch_segs[i]
            if len(t) < 20: continue
            
            # Find freeze point
            frozen_idx = len(act)
            for j in range(len(act) - 3):
                if np.max(act[j:j+3]) - np.min(act[j:j+3]) < 0.001 and np.max(tgt[j:j+3]) - np.min(tgt[j:j+3]) > 1.0:
                    frozen_idx = j
                    break
            
            if frozen_idx < 20: continue
            
            t = t[:frozen_idx]
            tgt = tgt[:frozen_idx]
            act = act[:frozen_idx]
            r = r[:frozen_idx]
            p = p[:frozen_idx]
            
            err = np.abs(act - tgt)
            max_err = np.max(err)
            
            if max_err > max_err_global:
                max_err_global = max_err
                best_seg_t = t
                best_seg_tgt = tgt
                best_seg_act = act
                best_seg_roll = r
                best_seg_pitch = p
                best_mid = mid

    return best_seg_t, best_seg_tgt, best_seg_act, best_seg_roll, best_seg_pitch, best_mid

plt.figure(figsize=(16, 18))

for i, (title, log_file) in enumerate(LOGS):
    t, tgt, act, roll, pitch, mid = get_valid_data(log_file)
    
    if len(t) > 0:
        t = t - t[0] # Normalize time
        
        # Plot 1: Tracking Error
        plt.subplot(4, 2, i*2 + 1)
        plt.title(f"{title} - Worst Calf Tracking (Motor {mid})", fontsize=12)
        plt.plot(t, tgt, 'r--', linewidth=1.5, label="Target")
        plt.plot(t, act, 'b-', linewidth=1.5, label="Actual")
        
        err = act - tgt
        max_err_idx = np.argmax(np.abs(err))
        max_err_val = err[max_err_idx]
        
        plt.annotate(f'Max Err: {max_err_val:.1f}°', 
                     xy=(t[max_err_idx], act[max_err_idx]), 
                     xytext=(t[max_err_idx], act[max_err_idx] + (20 if max_err_val < 0 else -20)),
                     arrowprops=dict(facecolor='black', shrink=0.05, width=1, headwidth=5),
                     fontsize=11, color='red', fontweight='bold')
                     
        plt.ylabel("Angle (deg)")
        plt.grid(True, linestyle=':', alpha=0.7)
        if i == 3: plt.xlabel("Time (s)")
        
        # Plot 2: IMU Roll & Pitch
        plt.subplot(4, 2, i*2 + 2)
        plt.title(f"{title} - IMU Posture (Roll & Pitch)", fontsize=12)
        plt.plot(t, roll, 'g-', linewidth=1.5, label="Roll (Sway)")
        plt.plot(t, pitch, 'm-', linewidth=1.5, label="Pitch (Tilt)")
        
        max_roll = np.max(np.abs(roll))
        max_pitch = np.max(np.abs(pitch))
        
        plt.annotate(f'Max Roll: {max_roll:.1f}°', xy=(0.05, 0.85), xycoords='axes fraction', color='green', fontweight='bold')
        plt.annotate(f'Max Pitch: {max_pitch:.1f}°', xy=(0.05, 0.75), xycoords='axes fraction', color='purple', fontweight='bold')
        
        plt.ylabel("Angle (deg)")
        plt.legend(loc='upper right', fontsize=10)
        plt.grid(True, linestyle=':', alpha=0.7)
        if i == 3: plt.xlabel("Time (s)")

plt.tight_layout()
plt.savefig("comprehensive_analysis.png", dpi=150)
print("Plot saved to comprehensive_analysis.png")
