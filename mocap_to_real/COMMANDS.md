# Marsdog Mocap-to-Real 常用命令

所有命令在机器人上执行，工作目录：
```bash
cd /userdata/marsdog_ws2/marsdogv3_ws/mocap_to_real
```

## 1. 设零位

摆好 URDF 零位姿态后执行（**含因克斯 2/3/6/7**；默认跳过头/颈 15–18）：

```bash
cd /home/cat/marsdog_v3_test3

# 演习（不写 Flash）
./run_with_env.sh python -m marsdog_control.apps.tools.calibration.set_zero_all --dry-run

# 整机设零（灵足 + EVO + 因克斯 + 达妙）
./run_with_env.sh python -m marsdog_control.apps.tools.calibration.set_zero_all

# 只设因克斯前腿
./run_with_env.sh python -m marsdog_control.apps.tools.calibration.set_zero_all --ids 2,3,6,7

# 跳过达妙 tarsus
./run_with_env.sh python -m marsdog_control.apps.tools.calibration.set_zero_all --no-dm
```

达妙无掉电零点记忆：即使脚本里设了，下次上电前仍需掰到硬限位。

## 2. 回零位

```bash
# 默认 3 秒渐变回零
python3 go_zero.py

# 5 秒渐变（更安全）
python3 go_zero.py --fade 5

# 低刚度（更柔顺）
python3 go_zero.py --kp-lz 8 --kd-lz 0.8
```

## 3. 动捕回放

```bash
# 保守测试（挂空中）
python3 replay.py --scale 0.2 --speed 0.3

# 半幅半速 + 日志记录
python3 replay.py --scale 0.5 --speed 0.5 --log

# 全幅全速
python3 replay.py --scale 1.0 --speed 1.0

# 循环播放
python3 replay.py --scale 0.5 --speed 0.5 --loop

# 只测试单腿（后左腿 7,8,9）
python3 replay.py --scale 0.3 --speed 0.5 --joints 7,8,9
```

回放中按键：
- `ESC` / `q` — 急停
- `+` / `-` — 调幅度 ±0.1
- `[` / `]` — 调速度 ±0.1
- `空格` — 暂停/恢复

## 4. 单关节测试

```bash
# 扫描所有电机
python3 test_single.py scan

# 正弦波测试单个电机（幅度 10°，周期 3s）
python3 test_single.py sine --id 1 --amp 10 --period 3

# 方向验证
python3 test_single.py direction --id 1
```

## 5. 典型工作流

```bash
# 1) 断电后重新设零（如果零位丢失）
python3 set_zero_all.py --ids 4,8

# 2) 回零位确认
python3 go_zero.py --fade 5

# 3) 小幅测试
python3 replay.py --scale 0.2 --speed 0.3

# 4) 逐步加大
python3 replay.py --scale 0.5 --speed 0.5 --log
```
## 6. 站立姿态 + 动捕回放（推荐流程）

```bash
# 站立 0.25m 体高 → CSV 回放, kp=5（挂空中先测）
cd /userdata/marsdog_ws2/marsdogv3_ws/mocap_to_real && python3 replay.py --scale 0.5 --speed 0.3 --height 0.25 --fade 3.0 --kp-lz 5 --kd-lz 1.0 --log

# 站立 0.25m, kp=15（换好电源后）
cd /userdata/marsdog_ws2/marsdogv3_ws/mocap_to_real && python3 replay.py --scale 0.5 --speed 0.3 --height 0.25 --fade 3.0 --kp-lz 15 --kd-lz 1.5 --log

# 站立 0.22m（更低蹲姿, 重心更稳）
cd /userdata/marsdog_ws2/marsdogv3_ws/mocap_to_real && python3 replay.py --scale 0.5 --speed 0.3 --height 0.22 --fade 3.0 --kp-lz 5 --kd-lz 1.0 --log

# 跳过站立, 直接 CSV frame0（旧行为）
cd /userdata/marsdog_ws2/marsdogv3_ws/mocap_to_real && python3 replay.py --scale 0.5 --speed 0.3 --no-stand --fade 3.0 --kp-lz 5 --kd-lz 1.0 --log
```

## 7. 运动学工具

```bash
# 查看 0.25m 站立姿态各电机角度
cd /userdata/marsdog_ws2/marsdogv3_ws/mocap_to_real && python3 kinematics.py --height 0.25

# 0.22m 低蹲姿态
cd /userdata/marsdog_ws2/marsdogv3_ws/mocap_to_real && python3 kinematics.py --height 0.22

# 前腿 hip 前倾 10°, 后腿 thigh 前倾 10°
cd /userdata/marsdog_ws2/marsdogv3_ws/mocap_to_real && python3 kinematics.py --height 0.25 --front-hip 10 --rear-thigh 10
```

## 8. 运动学步态控制（walk.py）

```bash
# ── 基础用法 ──────────────────────────────────────────────────────────
# 推荐: 默认参数 (kp=30, kd=4.0, 步幅~10cm, 站立后键盘切换)
cd /userdata/marsdog_ws2/marsdogv3_ws/mocap_to_real && python3 walk.py --log

# 启动即进入 trot，带日志（最常用）
cd /userdata/marsdog_ws2/marsdogv3_ws/mocap_to_real && python3 walk.py --trot --log

# 高刚度（参考老固件 kp_calf=60）
cd /userdata/marsdog_ws2/marsdogv3_ws/mocap_to_real && python3 walk.py --trot --kp-lz 50 --kd-lz 4.0 --log

# ── 步频 + 步幅调试 ───────────────────────────────────────────────────
# 慢速 trot（周期 1.0s），验证方向和姿态是否正确
cd /userdata/marsdog_ws2/marsdogv3_ws/mocap_to_real && python3 walk.py --trot --period 1.0 --kp-lz 30 --log

# 正常 trot（周期 0.6s，默认）
cd /userdata/marsdog_ws2/marsdogv3_ws/mocap_to_real && python3 walk.py --trot --period 0.6 --log

# 较快 trot（周期 0.4s）
cd /userdata/marsdog_ws2/marsdogv3_ws/mocap_to_real && python3 walk.py --trot --period 0.4 --log

# 大步幅（前/后腿各 15cm 步幅，参考用）
cd /userdata/marsdog_ws2/marsdogv3_ws/mocap_to_real && python3 walk.py --trot --amp-front 0.15 --amp-rear 0.15 --log

# ── 运行中键盘控制 ────────────────────────────────────────────────────
# SPACE/s  — 切换站立/Trot
# + / -    — 加快/减慢步频（调整周期）
# u / d    — 升高/降低体高 (±1cm)
# f / v    — 增大/减小摆幅 (±0.02rad)
# p        — 打印当前电机状态（使能/实际位置）
# q / ESC  — 退出（自动蹲下后失能）

# ── 离线仿真（不需要连接电机）────────────────────────────────────────
# 验证 trot 步态方向是否正确（看 hip_motor° 的符号变化规律）
cd /userdata/marsdog_ws2/marsdogv3_ws/mocap_to_real && python3 gait_controller.py --gait trot --height 0.25 --period 0.6 --steps 8

# 打印站立姿态目标角度
cd /userdata/marsdog_ws2/marsdogv3_ws/mocap_to_real && python3 gait_controller.py --gait stand --height 0.25
```

## 9. 带 kp 参数的一键命令（旧版, 不含站立）

```bash
# k p=5, scale=0.5, 带日志
cd /userdata/marsdog_ws2/marsdogv3_ws/mocap_to_real && python3 replay.py --scale 0.5 --speed 0.3 --fade 3.0 --kp-lz 5 --kd-lz 1.0 --no-stand --log

# 只测前右腿（验证符号修正）
cd /userdata/marsdog_ws2/marsdogv3_ws/mocap_to_real && python3 replay.py --scale 0.5 --speed 0.3 --joints 4,5,6 --no-stand --log

# 只测 Motor 9 (rl_calf, 排查硬件)
cd /userdata/marsdog_ws2/marsdogv3_ws/mocap_to_real && python3 replay.py --scale 1.0 --speed 0.3 --joints 9 --no-stand --log
```