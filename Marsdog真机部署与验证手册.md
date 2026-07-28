# Marsdog 真机部署与验证手册

本文档用于把当前 `20260714_1006` 控制程序放到开发板上做真机验证。分层重构已收官：
`src/marsdog_control` 自洽、`RuntimePipeline.tick` 为唯一稳态路径，真机行为与 legacy 逐字等价。

当前结论：

- **可以上真机。** 稳态由新 `RuntimePipeline` 接管，离线 golden 逐字节等价。
- **推荐入口 `python3 -m marsdog_control.apps.walk`（或 `./run_walk.sh`）**；
  `python3 mocap_to_real/walk.py` 为等价兼容入口。
- `--legacy-loop` / `--no-legacy-loop` 已废弃为空 flag，加不加都一样（见 2.2）。
- 重构目标是“行为不变、架构解耦”，不是改步态参数让狗走得更激进。
- 真机确认无回归后，才进入“调参数让它走得更好”的阶段。

---

## 1. 当前程序状态

当前项目已经完成了一批可离线验证的解耦工作：

- 独立模块已经下沉到 `src/marsdog_control/**`：
  - 电机驱动
  - IMU 驱动
  - 运动学
  - 步态控制器
  - IMU 姿态补偿控制器
  - 安全监督器
  - 运行时 FSM
  - 配置与单位转换
- **推荐真机入口(新)**：`python3 -m marsdog_control.apps.walk`（或 `./run_walk.sh`）。
  兼容入口 `python3 mocap_to_real/walk.py` 转发到同一实现，行为逐字等价。
- **稳态主循环已由新架构接管**：`RuntimeApp → RuntimePipeline.tick()`（= `tick_walk_loop`）
  是唯一稳态路径；bring-up 后编排在 `runtime/walk_session.py`。
- `--legacy-loop` 已退化为**空 flag**（无任何代码读取，仅为 CLI 向后兼容保留），
  开/关都不改变运行路径；`--no-legacy-loop` 同理，不再有特殊含义。
- **`src` 已自洽**：电机/IMU/手柄/运动学/步态/安全/FSM/契约检查全部下沉 `src/marsdog_control/**`，
  内部不再 import 旧扁平模块；`mocap_to_real/*.py` 仅剩兼容壳 + 诊断/分析工具 + 启动器。
- 离线测试保护：motion/executor parity、fake hardware full-loop parity（golden 逐字节）、
  输入层单测、日志初始化单测、ramps 单测、状态读取 fake motor 单测、
  DM 前瞻单测、`src` 自洽边界守卫。

开发机当前全量测试结果：

```text
PYTHONPATH=src:mocap_to_real python3 -m unittest discover -s tests -p "test_*.py"

Ran 54 tests
OK
```

这说明当前重构在离线层面没有发现行为漂移，但**不能替代真机验证**。

---

## 2. 上真机前的核心原则

### 2.1 第一次只验证“没回归”

第一次上开发板，不要追求“走得更好”，先确认：

- 能正常启动。
- 电机能正常探测。
- 上电姿态读取正常。
- 能平滑站起。
- 站立姿态和以前一致。
- 原来的 trot / natural trot / 调高 / IMU 开关行为没有变化。
- 没有多余抖动、抽腿、突然甩腿。

### 2.2 关于 `--legacy-loop`（已废弃为空 flag）

`RuntimePipeline.tick` 现已是唯一稳态路径，`--legacy-loop` / `--no-legacy-loop`
**不再有任何效果**（没有代码读取它），保留仅为兼容旧脚本。加不加都一样。

推荐入口：

```bash
python3 -m marsdog_control.apps.walk        # 或 ./run_walk.sh
```

兼容入口（等价实现）：

```bash
python3 mocap_to_real/walk.py
```

### 2.3 先架空/抱起验证，再落地

第一次运行建议分三步：

1. 狗悬空或支撑架上，轮/腿不接触地面。
2. 地面但人手扶住，低幅度验证。
3. 正常落地行走。

不要第一次就直接落地进入 trot。

### 2.4 调参和重构分开

当前版本的目标是：

- 代码结构更清楚。
- 参数更容易传。
- 单位更统一。
- 未来方便接 RL 和嵌入式驱动接口。
- 真机行为尽量保持原样。

如果上真机后觉得还需要“走得更好”，应该另起一轮调参，不要把调参和架构重构混在一起。

---

## 3. 推荐上板方式

### 3.1 推荐使用 Git 拉取

如果开发板能访问内网 GitLab，建议直接在开发板上拉取：

```bash
git clone http://192.168.1.10/jianwen.zheng/marsdogv3_test1.git
cd marsdogv3_test1
git checkout main
```

如果已经 clone 过：

```bash
cd marsdogv3_test1
git checkout main
git pull
```

### 3.2 如果使用拷贝方式

可以把整个 `20260714_1006` 目录拷到开发板，例如：

```bash
scp -r 20260714_1006 user@board_ip:/home/user/marsdog/
```

开发板上进入目录：

```bash
cd /home/user/marsdog/20260714_1006
```

### 3.3 保留旧版本

第一次上板前，建议不要覆盖开发板上原来能跑的版本。可以保留两个目录：

```text
marsdog_old_ok/
marsdog_refactor_test/
```

这样出问题时可以快速回退。

---

## 4. 开发板环境检查

### 4.1 Python 版本

建议使用 Python 3.10 或接近版本。

检查：

```bash
python3 --version
```

### 4.2 依赖检查

至少需要确认这些 Python 包/系统能力可用：

- `pyserial` 或项目内串口实现可正常工作
- CAN 设备可访问
- 串口设备有权限
- IMU 串口设备存在
- 电机总线设备存在

如果程序使用项目内自带的串口模块，也要注意不要和系统 `serial` 包冲突。

### 4.3 设备节点检查

运行前确认实际设备路径和配置一致。重点检查：

- LZ CAN A 设备
- LZ 串口/CAN B 设备
- EVO CAN 设备
- Damiao CAN 设备
- WT901 IMU 串口
- 手柄设备

配置位置主要在：

```text
mocap_to_real/bus_config.py
src/marsdog_control/config/bus_config.py
src/marsdog_control/config/devices.py
```

注意：`mocap_to_real/bus_config.py` 当前是兼容别名，真实实现已经在 `src/marsdog_control/config/bus_config.py`。

### 4.4 权限检查

如果串口或 CAN 没权限，可能需要：

```bash
sudo usermod -aG dialout $USER
sudo usermod -aG plugdev $USER
```

修改用户组后通常要重新登录。

如果临时验证，也可以先用 `sudo` 跑，但长期不建议依赖 `sudo`。

---

## 5. 上板前必须确认的硬件状态

### 5.1 机械安全

确认：

- 狗身周围没有人手、线缆、工具。
- 腿部没有卡住。
- 电机输出轴、连杆、轮子没有机械干涉。
- 狗可以被快速断电。
- 急停方式明确。

### 5.2 电机零位和方向

确认：

- LZ 电机 ID 和关节映射一致。
- EVO 电机 ID 和关节映射一致。
- 达妙 tarsus 电机 ID 一致。
- 关节正负方向没有因为换电机或改线而反。

如果刚换电机，先不要跑步态，先做：

- 单关节方向测试。
- 站姿目标小幅度检查。
- 趴下/站起路径检查。

### 5.3 达妙 tarsus 特别注意

达妙 S2325 没有掉电零点记忆。约定是：**每次上电前先手动把 tarsus 转到硬限位机械零点，再上电开机**。

程序侧：

- 已去掉 `--tarsus-zeroed` 开关（默认假定你已手动归零）。
- 配置了 natural / soft trot 时，全程主动驱动前腿 tarsus。
- 未配置自然步态时，tarsus 仍保持开机读到的角度（hold）。

---

## 6. 第一次运行建议命令

### 6.1 最保守：无手柄、无尾巴、不开日志

第一次只看能否启动、探测、站立：

```bash
python3 mocap_to_real/walk.py --no-gamepad --no-tail --no-log
```

说明：

- `--no-gamepad` 避免手柄输入误触发。
- `--no-tail` 避免尾巴通道干扰。
- `--no-log` 避免日志文件写入问题影响第一次启动。

### 6.2 开日志版本

确认基础正常后再打开日志：

```bash
python3 mocap_to_real/walk.py --no-gamepad --no-tail
```

日志默认写到：

```text
mocap_to_real/log/
```

### 6.3 手柄验证

确认站立稳定后再接手柄：

```bash
python3 mocap_to_real/walk.py --no-tail
```

手柄验证顺序：

1. 不碰摇杆，看是否保持站立。
2. START 是否能切换站立/走。
3. SELECT / B 是否能急停。
4. LB / RB 是否能改变步频。
5. 左摇杆 Y 是否控制前后。
6. 右摇杆 X 是否控制转向。

### 6.4 WBC 同参首跑（与仿真 NATURAL_SOFT_TROT_WBC 对齐）

仿真已验证的狗感小跑用同一套数字上机；**先记基线再微调**。完整清单见
[`docs/REAL_WBC_SAME_PARAMS_BRINGUP.md`](docs/REAL_WBC_SAME_PARAMS_BRINGUP.md)。

仿真 estimator 冒烟：

```bash
./scripts/estimator_wbc_smoke.sh
```

真机同参（吊带 / 急停就绪后）：

```bash
./run_walk.sh --natural-soft-trot --wbc --no-vmc --base-estimate-mode estimator
```

要求：

- 控制路径只用 `--base-estimate-mode estimator`（不要用 `truth`）。
- 对照 `docs/baselines/sim_wbc_estimator_summary.json` 与真机 `telemetry_summary` /
  日志中的 roll、vx_est−cmd、contact mismatch、q_err。

---

## 7. 真机验证流程

### 7.1 阶段 A：导入/启动验证

目标：程序能启动，不因为路径、依赖、设备节点报错。

执行：

```bash
python3 mocap_to_real/walk.py --no-gamepad --no-tail --no-log
```

观察：

- 是否能加载配置。
- 是否能初始化电机驱动。
- 是否能初始化 IMU。
- 是否有 fatal 配置错误。
- 是否有设备不存在。

如果这里失败，先不要继续，优先修：

- 设备路径
- 权限
- 依赖
- CAN/串口连接

### 7.2 阶段 B：电机探测与当前位置读取

目标：确认电机在线、位置读取正常。

观察启动打印：

- 哪些电机在线。
- 哪些电机失能。
- 哪些电机 fault。
- 开机读取位置是否异常。

如果某个关节角度明显不对：

- 先检查 ID。
- 再检查电机方向。
- 再检查机械零位。
- 不要直接进入 trot。

### 7.2.1 单/多电机跟随诊断（`tests/Motor_test`）

当 walk 日志里某关节跟随差（如小腿 3/7、前掌 4/8），或需要验证 **与 walk 相同的驱动下发路径** 时，用本目录脚本，**不要**先改 gait PID。

要点：

- 路径：`tests/Motor_test/`（详细说明见同目录 `README.md`）。
- **不是 unittest**：接真电机；勿用 `python -m unittest discover -s tests` 跑这里。
- 下发走 `RkMotorBoard.send_angles` → mapping → lz/evo/dm/incos，与 walk 同源；**不含 gait / FSM / IMU**。
- 狗须**悬空**；只激励 `--ids`，其余在线轴保持开机角；目标相对 `q0` 小幅度运动。

工程根目录：

```bash
export PYTHONPATH=src:mocap_to_real

# 探测
python3 tests/Motor_test/bench_motor_track.py probe --ids 3,7,4,8

# 保持（使能/反馈）
python3 tests/Motor_test/bench_motor_track.py hold --ids 4,8 --kp 40 --kd 2 --sec 5

# 阶跃（证达妙 4/8 是否能动）
python3 tests/Motor_test/bench_motor_track.py step --ids 4,8 --deg 3 --kp 40 --sec 2

# 正弦（因克斯 3/7 跟随）
python3 tests/Motor_test/bench_motor_track.py sine --ids 3,7 --amp-deg 5 --period 2 \
  --kp 50 --kd 1 --hz 200 --sec 10

# 离线画跟随曲线
python3 tests/Motor_test/plot_tracking.py --latest --error --motors 3,7,4,8
```

日志与出图默认在 `tests/Motor_test/log/`（`bench_motor_{mode}_{ts}.csv` / `*_tracking.png`）。

建议排查顺序：`probe` → `hold` → `step`（先证能动）→ `sine`（再谈 PID）；左右轴可分开 `--ids` 对照。

### 7.3 阶段 C：悬空站立验证

目标：确认站起路径没有抽腿。

建议：

- 狗悬空或支撑架上。
- 人手可快速断电。
- 先不开手柄。

观察：

- 站起是否平滑。
- 有没有单腿突然甩动。
- 有没有某个关节目标方向反了。
- tarsus 是否保持开机角度。
- 站立后有没有持续高频抖动。

异常处理：

- 任何抽腿/反向/撞限位，立即停止。
- 不要试图通过加大 kp 解决方向错误。

### 7.4 阶段 D：落地站立验证

目标：确认狗落地后能稳定站住。

观察：

- 身体是否塌陷。
- 高度是否接近设定。
- 四腿受力是否明显不均。
- 是否有左右倾斜。
- IMU 补偿开启/关闭时差异是否合理。

建议先跑：

```bash
python3 mocap_to_real/walk.py --no-gamepad --no-tail
```

如需关闭 IMU 补偿，按当前 CLI 参数使用对应开关。具体以 `--help` 输出为准：

```bash
python3 mocap_to_real/walk.py --help
```

### 7.5 阶段 E：低风险步态验证

目标：确认原有步态无回归。

建议顺序：

1. 原地站立。
2. 小幅前进。
3. 小幅后退。
4. 原地转向。
5. 调身高。
6. 切换步频。

不要一上来就最大速度。

### 7.6 阶段 F：日志回看

如果开启日志，重点看：

- `target_deg`
- `actual_deg`
- `error_deg`
- `torque_nm`
- `actual_kp`
- `actual_kd`
- `imu_roll_deg`
- `imu_pitch_deg`
- `imu_dz_*`
- `dm_feedback_age_ms`
- `dm_dropped_commands`

如果出现：

- 某个关节误差持续很大
- torque 接近 0 但误差很大
- IMU age 很大
- DM dropped commands 增加

不要继续跑步态，先排查通信和驱动。

---

## 8. 重点风险点

### 8.1 不要误用 `--no-legacy-loop`

当前可上真机的是 legacy 主循环路径。`RuntimePipeline` 还没有完成真机接管。

### 8.2 不要随便打开 active tarsus

前腿 tarsus 达妙电机必须明确物理归零后才允许主动控制。

### 8.3 不要把“高度塌陷”直接归因于 kp

身高被压下来可能来自：

- kp/kd 不足
- 目标高度超出机械/力矩实际能力
- 腿部几何接近极限
- 重力补偿没开或比例不合适
- IMU 补偿/trim 叠加后改变了目标脚高
- 电机限流/掉使能/通讯问题

应先看日志中的：

- target vs actual
- torque
- error
- enabled/fault

再决定是不是调 kp。

### 8.4 不要在错误方向上调大增益

如果关节方向反了，调大 kp 会更危险。必须先确认方向。

### 8.5 不要一次改多个参数

真机调参时，每次只改一类：

- 先高度
- 再 kp/kd
- 再步态周期
- 再摆幅
- 再 IMU 补偿
- 最后重力补偿

---

## 9. 当前还有哪些没做

### 9.1 `RuntimePipeline` 尚未接管主循环

当前状态：

- `RuntimePipeline` 结构已经存在。
- 但真机主循环仍由 `mocap_to_real/walk.py` 管理。
- `--no-legacy-loop` 不能作为首次真机路径。

原因：

- 主循环里包含实时硬件时序。
- IMU soft-start / D ramp / phase gate / auto-trim 与步态相位强耦合。
- shutdown / smooth transition / recover 只能真机验证。

后续怎么做：

1. 先在真机上确认当前 legacy 默认路径无回归。
2. 使用 full-loop fake hardware parity 作为离线基线。
3. 把主循环逐段搬进 `RuntimePipeline.tick()`。
4. 每搬一段，先跑 full-loop parity。
5. 上真机用 `--legacy-loop` 与 `--no-legacy-loop` 对比。
6. 确认站立、步态、调高、急停都一致后，再考虑默认切 pipeline。

### 9.2 `write_log()` 仍在 `walk.py`

当前只搬了 `setup_log()`。

`write_log()` 还在 `walk.py`，原因是它读取大量 live 状态：

- LZ/EVO 实际位置
- DM timing
- torque
- IMU components
- foot pitch 重建
- `_resolve_gains`
- `_REAL_JOINTS`
- `DM_FIXED_TARGETS`

后续怎么做：

1. 定义 `LogRuntime` 扩展字段。
2. 把 `front_foot_pitch_from_motor()` 先搬出去。
3. 把 `write_log()` 行构造拆成纯函数。
4. 用 fake motor 写一行 CSV 做离线对比。
5. 再替换 `walk.write_log()` 为薄封装。

### 9.3 `check_motors()` / `smooth_transition()` / 恢复逻辑仍在 `walk.py`

这些函数包括：

- `check_motors()`
- `find_lz_recoverable_faults()`
- `recover_lz_stand_faults()`
- `smooth_transition()`
- EVO re-enable
- shutdown ramp

原因：

- 它们是启动/恢复/关机的真实硬件 I/O 时序。
- 离线假电机只能验证语法和调用顺序，不能证明真实电机安全。

后续怎么做：

1. 先真机记录当前启动、站起、趴下、关机行为。
2. 把这些函数搬到 `runtime/startup.py`、`runtime/shutdown.py`、`hardware/robot_hw.py`。
3. `walk.py` 保留薄封装。
4. 真机逐项对比：
   - 站起时间
   - 站姿最终角度
   - 关机趴下路径
   - 恢复逻辑是否误判

### 9.4 真机调参还没开始

当前重构没有主动改步态参数。

后续想让狗走得更好，需要单独做调参：

- kp/kd
- body height
- gait period
- front/rear amplitude
- turn gain
- IMU kp/kd/i
- IMU trim
- gravity compensation scale
- variable impedance 参数

调参必须基于日志和真机现象，不要盲调。

---

## 10. 建议真机验证记录表

每次运行建议记录：

```text
日期:
代码版本/commit:
命令:
是否悬空:
是否开 IMU:
是否开日志:
是否开手柄:
是否 active tarsus:
现象:
异常关节:
日志文件:
是否可复现:
下一步:
```

示例：

```text
日期: 2026-07-16
代码版本: main / <commit>
命令: python3 mocap_to_real/walk.py --no-gamepad --no-tail
是否悬空: 是
是否开 IMU: 是
是否开日志: 是
是否开手柄: 否
是否 active tarsus: 否
现象: 平滑站起，右后小腿轻微抖动
异常关节: rr_calf
日志文件: mocap_to_real/log/walk_log_xxx.csv
是否可复现: 是
下一步: 看 rr_calf target/actual/torque，确认是否通信或 kp 问题
```

---

## 11. 推荐第一次真机验证 checklist

### 上电前

- [ ] 狗固定在安全位置。
- [ ] 急停/断电方式明确。
- [ ] 电池电压正常。
- [ ] CAN/串口线缆固定。
- [ ] 达妙 tarsus 上电前已手动归零（程序不再提供 `--tarsus-zeroed` 确认开关）。
- [ ] 开发板上保留旧版本程序。

### 程序启动前

- [ ] `python3 --version` 正常。
- [ ] 设备节点存在。
- [ ] 用户有串口/CAN 权限。
- [ ] `git pull` 或文件拷贝完成。
- [ ] 当前目录正确。

### 第一次启动

- [ ] 使用 `--no-gamepad --no-tail --no-log`。
- [ ] 不使用 `--no-legacy-loop`。
- [ ] 不使用 active tarsus。
- [ ] 不直接 `--trot` 起步。

推荐命令：

```bash
python3 mocap_to_real/walk.py --no-gamepad --no-tail --no-log
```

### 站立验证

- [ ] 无抽腿。
- [ ] 无撞限位。
- [ ] 身体高度合理。
- [ ] 左右不明显倾斜。
- [ ] 电机没有持续 fault。
- [ ] IMU 数据不是 stale。

### 步态验证

- [ ] 小幅前进正常。
- [ ] 小幅后退正常。
- [ ] 转向正常。
- [ ] 调高度正常。
- [ ] 急停正常。
- [ ] 关机趴下正常。

### WBC 同参首跑（可选，站立/慢走 OK 之后）

- [ ] 使用 `./run_walk.sh --natural-soft-trot --wbc --no-vmc --base-estimate-mode estimator`。
- [ ] 未开 `--base-estimate-mode truth`。
- [ ] 吊带保护；小油门。
- [ ] 保存遥测并与 `docs/baselines/sim_wbc_estimator_summary.json` 对照（见 6.4）。

---

## 12. 出问题时怎么回退

### 12.1 运行中异常

立即：

1. 急停或断电。
2. 不要继续 trot。
3. 保存日志。
4. 记录命令和现象。

### 12.2 程序启动失败

优先检查：

- 设备路径
- 权限
- 依赖
- Python 版本
- 当前目录

### 12.3 行为和旧版不一致

处理方式：

1. 用旧版本同样命令跑一次。
2. 用当前版本同样命令跑一次。
3. 对比：
   - 启动打印
   - 电机在线列表
   - 站姿角度
   - 日志 target/actual
4. 如果当前版本异常，先回退到旧版本，不要现场继续大改。

### 12.4 快速回退方案

如果是 Git 拉取：

```bash
git log --oneline
git checkout <旧的可用commit>
```

如果保留了旧目录：

```bash
cd ../marsdog_old_ok
python3 mocap_to_real/walk.py ...
```

---

## 13. 后续推荐工作顺序

### 第一步：当前版本上真机复现

目标：确认重构后默认 legacy 主循环无回归。

必须完成：

- 站立
- 趴下
- trot
- 调身高
- IMU 开/关
- 手柄
- 急停

若 walk 跟随曲线显示个别关节异常，先用 **§7.2.1 `tests/Motor_test`** 做无 gait 单轴/多轴对照，再进入调参。

### 第二步：基于日志做调参

目标：让狗实际走得更好。

建议顺序：

1. 先解决明显硬件/方向/通信问题（可用 `tests/Motor_test` 复现）。
2. 再调站立高度和 kp/kd。
3. 再调步态周期和摆幅。
4. 再开 IMU 补偿。
5. 最后调重力补偿和 variable impedance。

### 第三步：搬 startup/shutdown/diagnostics

目标：把剩余硬件 I/O 时序从 `walk.py` 拆出。

必须有真机对比：

- 站起路径
- 趴下路径
- fault 恢复
- EVO re-enable

### 第四步：`RuntimePipeline` 接管

目标：让 `RuntimePipeline.tick()` 成为主循环。

做法：

1. 先用 fake hardware full-loop parity 对齐命令流。
2. 再真机 `--legacy-loop` 和 `--no-legacy-loop` 对比。
3. 只在确认一致后改默认入口。

### 第五步：移除 legacy shim

只有当真机确认 `RuntimePipeline` 完全可用后，才考虑：

- 移除 `--legacy-loop`
- 移除 `mocap_to_real` 兼容别名
- 把 `walk.py` 退化为 CLI 薄壳

---

## 14. 最重要的结论

当前版本可以上真机，但上真机的目的应该是：

1. **确认重构没有引入真机行为回归。**
2. **用日志找真实调参方向。**
3. **为后续 `RuntimePipeline` 接管提供真机基线。**

第一次上板推荐命令：

```bash
python3 mocap_to_real/walk.py --no-gamepad --no-tail --no-log
```

确认无异常后再逐步打开：

```bash
python3 mocap_to_real/walk.py --no-gamepad --no-tail
python3 mocap_to_real/walk.py --no-tail
```

暂时不要第一次就用：

```bash
python3 mocap_to_real/walk.py --no-legacy-loop
```

也不要第一次就打开 active tarsus / aggressive trot 参数。

**先复现，再调参，再切 pipeline。**
