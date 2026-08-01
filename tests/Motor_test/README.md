# Motor_test — 真机电机跟随 bench

**不是 unittest。** 本目录脚本接真实电机，通过 `RkMotorBoard.send_angles` 走与 `walk` 相同的驱动下发路径（mapping → lz/evo/dm/incos），**不含 gait / FSM / IMU**。

勿用 `python -m unittest discover -s tests` 跑这里的文件（脚本不以 `test_` 开头，一般不会被收集）。

## 安全

- 狗必须**悬空 / 支架支撑**，人手可断电。
- 只激励 `--ids` 列出的电机；其余在线轴每拍保持开机角度。
- 目标始终相对开机位置 `q0`，默认小幅度（step/sine 约 ±3°）。

## 依赖

```bash
# 工程根目录
export PYTHONPATH=src:mocap_to_real   # mocap_to_real 仅为当前驱动兼容 shim
```

## 用法

```bash
# 探测
python3 tests/Motor_test/bench_motor_track.py probe --ids 3,7,4,8

# 保持（使能/反馈）
python3 tests/Motor_test/bench_motor_track.py hold --ids 4,8 --kp 40 --kd 2 --sec 5

# 阶跃（证达妙 4/8 是否能动）
python3 tests/Motor_test/bench_motor_track.py step --ids 4,8 --deg 3 --kp 40 --sec 2

# 正弦（因克斯 3/7 跟随 / PID）
python3 tests/Motor_test/bench_motor_track.py sine --ids 3,7 --amp-deg 5 --period 2 \
  --kp 50 --kd 2.5 --hz 200 --sec 10

# 前腿四个因克斯 2/3/6/7 网格扫频（须吊起；对照 ENCOS 力位混控 KP≤500 KD≤5）
./run_with_env.sh python tests/Motor_test/sweep_incos_front.py \
  --kp 15,25,35,45 --kd 0.8,1.5,2.5 --amp-deg 3 --sec 5
```

日志默认：`tests/Motor_test/log/bench_motor_{mode}_{ts}.csv`

## 画跟随曲线

```bash
python3 tests/Motor_test/plot_tracking.py --latest --error --motors 3,7,4,8
# 或
python3 tests/Motor_test/plot_tracking.py tests/Motor_test/log/bench_motor_sine_XXXX.csv \
  --error --motors 3,7
```

## 与 walk 的关系

| 项 | 本 bench | walk |
|---|---|---|
| 下发 | `RkMotorBoard.send_angles` | 同（经 `walk.send_all` → board） |
| Mapping | `build_board_command_batches` | 同 |
| 达妙 active | 测试 ID 含 dm 时 `dm_tarsus_active=True` | natural soft trot 默认启用 |
| 轨迹 | hold / step / sine | gait / FSM |
