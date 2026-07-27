import csv
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.font_manager as fm

# 设置中文字体 (Jetson 上通常有文泉驿或 Droid Sans Fallback)
# 如果没有，matplotlib 会退回默认字体但可能显示方块
plt.rcParams['font.sans-serif'] = ['DejaVu Sans', 'Droid Sans Fallback', 'WenQuanYi Micro Hei', 'SimHei']
plt.rcParams['axes.unicode_minus'] = False

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

# 读取前腿小腿 (Motor 3) 和 前腿大腿 (Motor 2)
t_calf, tgt_calf, act_calf = read_motor_data(LOG, 3)
t_thigh, tgt_thigh, act_thigh = read_motor_data(LOG, 2)

if len(t_calf) > 0:
    # 选取开始奔跑后的一小段窗口 (2秒钟)，这样曲线不拥挤，看得更清楚
    t_start = t_calf[0] + 0.5
    mask_c = (t_calf >= t_start) & (t_calf <= t_start + 2.0)
    t_c, tgt_c, act_c = t_calf[mask_c], tgt_calf[mask_c], act_calf[mask_c]
    
    mask_t = (t_thigh >= t_start) & (t_thigh <= t_start + 2.0)
    t_t, tgt_t, act_t = t_thigh[mask_t], tgt_thigh[mask_t], act_thigh[mask_t]

    plt.figure(figsize=(12, 8))
    
    # 绘制小腿 (Calf)
    plt.subplot(2, 1, 1)
    plt.title("前腿小腿 (fl_calf) - 目标角度 vs 实际角度 (0.4秒极快步态)", fontsize=14)
    plt.plot(t_c, tgt_c, 'r--', linewidth=2, label="规划目标值 (Target)")
    plt.plot(t_c, act_c, 'b-', linewidth=2, label="电机实际值 (Actual)")
    
    # 标注最大误差处
    err_c = act_c - tgt_c
    max_err_idx = np.argmax(np.abs(err_c))
    plt.annotate(f'最大误差: {err_c[max_err_idx]:.1f}°\n(腿被严重压弯)', 
                 xy=(t_c[max_err_idx], act_c[max_err_idx]), 
                 xytext=(t_c[max_err_idx]-0.3, act_c[max_err_idx]-15),
                 arrowprops=dict(facecolor='black', shrink=0.05, width=1.5, headwidth=8),
                 fontsize=12, color='red')

    plt.ylabel("角度 (度)", fontsize=12)
    plt.legend(loc='upper right', fontsize=12)
    plt.grid(True, linestyle=':', alpha=0.7)

    # 绘制大腿 (Thigh Roll)
    plt.subplot(2, 1, 2)
    plt.title("前腿大腿侧摆 (fl_thigh_roll) - 目标角度 vs 实际角度", fontsize=14)
    plt.plot(t_t, tgt_t, 'r--', linewidth=2, label="规划目标值 (Target)")
    plt.plot(t_t, act_t, 'b-', linewidth=2, label="电机实际值 (Actual)")
    
    err_t = act_t - tgt_t
    max_err_t_idx = np.argmax(np.abs(err_t))
    plt.annotate(f'最大误差: {err_t[max_err_t_idx]:.1f}°', 
                 xy=(t_t[max_err_t_idx], act_t[max_err_t_idx]), 
                 xytext=(t_t[max_err_t_idx]-0.3, act_t[max_err_t_idx]+15),
                 arrowprops=dict(facecolor='black', shrink=0.05, width=1.5, headwidth=8),
                 fontsize=12, color='red')

    plt.xlabel("时间 (秒)", fontsize=12)
    plt.ylabel("角度 (度)", fontsize=12)
    plt.legend(loc='upper right', fontsize=12)
    plt.grid(True, linestyle=':', alpha=0.7)

    plt.tight_layout()
    plt.savefig("tracking_analysis_clean.png", dpi=150)
    print("Plot saved to tracking_analysis_clean.png")
else:
    print("No data found.")
