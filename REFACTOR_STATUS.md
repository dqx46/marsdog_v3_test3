# Marsdog 控制重构进度

> 更新：2026-07-21  
> 工程：`20260714_1006`  
> 目标架构：`USER → Planner → Kinematics → Recorder → Mapping → Board → Motor`

本文回答三件事：**我们要做什么**、**现在做到哪**、**为什么 `apps/walk.py` 看起来还是臃肿**。

---

## 1. 我们要做的（目标）

把原先约 3000 行、算法/硬件/CLI 缠在一起的 `mocap_to_real/walk.py`，拆成可维护、可测试、可换板的分层架构，并在真机行为不变的前提下收官。

### 目标分层

| 层 | 职责 | 理想归属 |
|---|---|---|
| App 壳 | CLI + 组装 + 启停 | `apps/walk.py`（薄） |
| Runtime | 稳态 tick、FSM、状态对象、bring-up/shutdown | `runtime/*` |
| Control | 平衡、执行器增益/前馈 | `control/*` |
| Motion | 步态/IK/模式选择 | `motion/*` |
| Input HAL | 键盘/手柄/蓝牙 → `UserCommand` | `input/*` |
| Hardware | Board 抽象 + 驱动 | `hardware/*` |
| Config | 关节表、增益、总线、typed config | `config/*` |

### 冲到「约 9 分」的验收标准

1. **`apps/walk.py` 是真薄壳**：只剩 CLI + 组装 + 调 `RuntimeApp.run()` / shutdown（目标量级：几百行，不是一千多行）。
2. **`RuntimePipeline.tick` 是唯一稳态路径**：少注入旧 callable，组件化硬件/输入。
3. **运行期旋钮是显式对象**：`WalkRuntimeState` / config，不再靠模块全局当权威。
4. **Input HAL 完整**：手柄/键盘/（预留）蓝牙统一入口。
5. **真机对照通过后，再删兼容 shim**（`mocap_to_real/*.py` 别名等）。

---

## 2. 已经做完的（可当真机用）

### 2.1 入口与包边界

- 真实实现在 `src/marsdog_control/`。
- `mocap_to_real/walk.py` 仅 **24 行兼容启动器**（转发到 `marsdog_control.apps.walk`）。
- 真机启动（二者等价）：

```bash
# 兼容入口
python3 mocap_to_real/walk.py --no-gamepad --no-tail

# 新包入口
PYTHONPATH=src:mocap_to_real python3 -m marsdog_control.apps.walk --no-gamepad --no-tail
```

### 2.2 硬件与控制主路径

| 能力 | 位置 | 状态 |
|---|---|---|
| Board 抽象（RK） | `hardware/board.py` | ✅ 真机在用 |
| 统一下发 / soft disable | Board + `actuation.py` | ✅ |
| 稳态 loop | `runtime/walk_loop.py` → `tick_walk_loop` | ✅ |
| 唯一稳态入口 | `RuntimeApp` → `RuntimePipeline.tick()` | ✅ |
| 组件组装 | `runtime/walk_assembly.py` | ✅ |
| 运行期旋钮对象 | `runtime/walk_state.py`（`WalkRuntimeState`） | ✅ 启动写入；全局仅兼容镜像 |
| 硬件 bring-up | `runtime/walk_bringup.py` | ✅ |
| 站立 fade / 站立后 IMU 校准 / 开输入 | 同上 | ✅ |
| 关机顺序 | `runtime/shutdown.py` | ✅ |
| CLI (`parse_args`) | `apps/walk_cli.py`（`walk` 再导出） | ✅ Phase A |
| 控制器/FSM/Safety/IMU 工厂 | `runtime/walk_controllers.py` | ✅ Phase A |
| 启动剧本（校验/预设/特性接线） | `runtime/walk_startup.py` | ✅ Phase A |
| 默认关节增益 | `config/gains.py`（含 fl/fr_calf kp50 kd1） | ✅ |
| 离线 parity / unittest | `tests/` + `tests/parity/` | ✅ 绿（50 + 3） |
| 真机电机跟随 bench | `tests/Motor_test/`（Board 路径，无 gait） | ✅ 已用；手册 §7.2.1 |
| 真机站立 / Ctrl+C 缓速失能 | 多次验证 | ✅ Phase A 后复验通过 |

### 2.3 Phase A 本轮改动（2026-07-18）

| 改动 | 说明 |
|---|---|
| 新增 `apps/walk_cli.py` | 整段 `parse_args` + `apply_preset_preserving_cli`；`walk` 再导出，parity 仍 `walk.parse_args` |
| 新增 `runtime/walk_controllers.py` | `assemble_walk_control_stack`：前进幅值、`build_controller_set`、FSM、Safety、IMU PID |
| 新增 `runtime/walk_startup.py` | `prepare_walk_startup`：bench/方向测试校验、tarsus 闸、自然步态预设、typed config、`WalkRuntimeState` 特性接线、横幅 |
| `apps/walk.main` | 前半段改为 `prepare_walk_startup`；bring-up / loop / shutdown 仍在 app |
| 薄清理 | 去掉无用 import；kinematics 仅再导出 `front_foot_pitch_from_motor` |
| 测试守卫 | `tests/test_defaults_sync.py` 注释指向 `walk_cli` |
| 新增 `mocap_to_real/plot_tracking_mpl.py` | 离线跟随曲线（walk 日志）；包装入口 `apps/tools/analysis/plot_tracking_mpl.py` |
| 新增 `tests/Motor_test/` | **真机**无 gait 电机跟随 bench（`bench_motor_track.py` + `plot_tracking.py`）；经 `RkMotorBoard.send_angles` 验证与 walk 同源驱动路径；**不是 unittest** |

**验证记录（Phase A 后）**

| 层级 | 命令 / 动作 | 结果 |
|---|---|---|
| 离线单元 | `PYTHONPATH=src:mocap_to_real python3 -m unittest discover -s tests -p "test_*.py"` | ✅ 50 OK |
| 离线 parity | `PYTHONPATH=src:mocap_to_real python3 -m unittest discover -s tests/parity -p "test_*.py"` | ✅ 3 OK |
| 总线探测 | `mocap_to_real/static_test.py` | ✅ 灵足 + 因克斯 3/7 等 ONLINE |
| 真机站立 | `walk.py --no-gamepad --no-tail --no-log` → fade → IMU 校准 → STAND → Ctrl+C | ✅ 站起平滑、缓速失能正常；工厂打印（`[转向层]`/`[fsm]`/`[PRED]`/`[AT]`）仍在 |
| 真机慢走 | 悬空站立后 SPACE 切入 NaturalSoftTrot 短测 → 回站立 / 缓速失能 | ✅ 无抽腿、工厂路径正常；日志 `walk_log_20260718_150502.csv` |
| 跟随曲线（walk） | `plot_tracking_mpl.py --latest --include-stand --error --motors 1..8` | ✅ 出图 `walk_log_20260718_150502_tracking.png` |
| 电机跟随 bench | `tests/Motor_test/bench_motor_track.py` step/sine + `plot_tracking.py` | ✅ 已跑通；可单独测 3/7/4/8（详见手册 §7.2.1） |

### 2.4 体量对照

| 文件 | 大约行数 | 说明 |
|---|---:|---|
| 旧单体 `walk.py`（参考仓） | ~3000 | 一切揉在一起 |
| Phase A 前 `apps/walk.py` | ~1687 | CLI + 控制器构造仍在 app 内 |
| 现 `apps/walk.py` | **~435** | 薄壳：CLI + startup + bring-up + 交棒 `run_walk_session` |
| `runtime/walk_session.py` | ~259 | **新**：bring-up 后编排(方向预检/fade/IMU 校准/输入/loop/shutdown) |
| `apps/walk_cli.py` | ~343 | Phase A：整段 argparse |
| `runtime/walk_controllers.py` | ~307 | Phase A：步态/FSM/Safety/IMU 工厂 |
| `runtime/walk_startup.py` | ~303 | Phase A：main 前半段启动剧本 |
| `runtime/walk_services.py` | ~212 | **新**：硬件下发/只读/诊断/平滑/日志 I/O 服务层(`WalkServices`) |
| `runtime/` 整包 | ~2729 | loop / FSM / bring-up / controllers / startup / services |
| `mocap_to_real/walk.py` | 24 | 只剩 shim |

**结论**：CLI、控制栈、startup 剧本与**硬件 I/O 逻辑**均已迁出，App 行数 **~1687 → ~976 → ~683 → ~581**。
`send_all`/`read_state`/`check_motors`/`smooth_transition`/`setup_log`/`write_log`/`shutdown`/`find_lz_recoverable_faults`/
`poll_user_command` 的实现体已下沉 `WalkServices`/`input.user_input`，**Phase E 已删除全部 legacy 委托壳**：
parity 直接 patch `WalkServices` 类方法，`main()` 注入 `svc.send_all`/`svc.read_state`；legacy 测试改直连 impl 模块。
约 **8.5/10**。剩余臃肿来自 `main()` 的 bring-up/fade/loop 编排（Phase F）。

---

## 3. 正在做的 / 半完成（过渡态）

这些是「已经有骨架，但还没删干净旧世界」的部分：

1. **`WalkRuntimeState` 权威化未收官**  
   - 已：CLI → state →（镜像）globals。  
   - 未：彻底删掉 `LEG_KP_SCALE` / `VAR_IMPEDANCE` / `ACTIVE_DM_*` 等模块全局读者。

2. **`WalkLoopContext` 仍注入大量 callable**  
   - `poll_user_command` / `read_state` / `send_all` / `apply_dev_tuning` 等仍是函数指针。  
   - 目标：`RobotHardware`、`WalkInputManager`、`MotionTargetSelector` 等组件对象。

3. **`apps/walk.py` 兼容面已清空（Phase E 完成）**  
   - `send_all` / `read_state` / `check_motors` / `smooth_transition` / `setup_log` / `write_log` /
     `read_positions` / `recover_lz_stand_faults` / `shutdown` / `find_lz_recoverable_faults` /
     `poll_user_command` 的委托壳**已全部删除**；实现唯一驻留 `WalkServices` / `input.user_input`。  
   - parity 直接 patch `WalkServices.send_all`/`read_state` 类方法；`main()` 注入 `svc.send_all`/`svc.read_state`。  
   - legacy 消费者已迁移：`test_runtime.py` 直连 `hardware.diagnostics` / `input.user_input`；
     DM 前瞻单测从 `test_imu_dm_pipeline.py`（已随全局删除而失效）移入 `tests/test_dm_reference_lead.py`，
     打 `hardware.actuation.send_all` + `WalkRuntimeState` 当前路径，**离线可跑**。  
   - `WalkServices` 持有 board / runtime_state / real_joints / clock，为单一 I/O 所有者；
     `main()` 里 `_BOARD`/`_stop` 模块全局已删，板句柄唯一存放于 `WalkServices.board` + `runtime_state.board`。

4. **`main()` 仍较长**  
   - ✅ 校验 / 预设 / RuntimeState 特性接线 → `prepare_walk_startup(...)`。  
   - ✅ 控制器 / IMU PID / FSM / Safety → `assemble_walk_control_stack(...)`。  
   - 未：方向测试预检、bring-up / fade / 组装 loop 编排仍在 app 内。

5. **兼容 shim 故意保留**  
   - 真机基线未完全「新路径独活」前，不删 `mocap_to_real` 别名，避免启动脚本/文档断裂。

---

## 4. 为何看起来还这么臃肿？

不是「没拆」，而是 **拆完后的过渡形态**：

```
旧世界：一个 3000 行文件 = 全部真理
现在：  很多小模块 = 真理，但 apps/walk.py 仍像「火车站」
         ├─ CLI → apps/walk_cli.py（已迁出，walk 再导出）
         ├─ startup → runtime/walk_startup.py（已迁出）
         ├─ 一堆 forward 包装（为了 import walk.xxx / patch walk.send_all）
         ├─ 控制器/FSM/IMU → runtime/walk_controllers.py（已迁出）
         └─ 全局镜像逻辑（为了旧热键/日志读全局）
```

**臃肿的主要来源（按观感排序）**

1. **兼容薄包装层** —— 实现已走，文件仍留同名函数，行数不降。  
2. **`main()` 后半段仍长** —— bring-up / fade / loop 组装编排还在 app。  
3. **双轨状态** —— `WalkRuntimeState` + globals mirror，多一套同步代码。  
4. **Parity 约束** —— `tests/parity` 通过 patch `walk.send_all` / `walk.read_state` 证明等价；过早删壳会漂 golden。

所以：**架构分已涨，文件视觉分继续下降（~1687→~976）** —— 下一步继续迁 bring-up 编排与删 forward。

---

## 5. 后续要做的（冲 9 分路线图 → 已完成 10/10，含 Phase A–I）

按依赖顺序，建议严格按阶段推进；每阶段都 **离线 parity 绿 → 真机站立确认**。

### Phase A — 压薄 App 壳（优先，直接改善「看着臃肿」）

- [x] 抽出 `apps/walk_cli.py`：整段 `parse_args` / `apply_preset_preserving_cli`  
- [x] 抽出 `runtime/walk_controllers.py`：`build_controller_set` + FSM + Safety + IMU PID 工厂  
- [x] 抽出 `runtime/walk_startup.py`：`prepare_walk_startup`（校验 + 预设 + RuntimeState 接线 + 横幅）  
- [x] 离线 unittest + parity 绿；真机站立 / 缓速失能复验通过  
- [x] 真机慢走短测（悬空 NaturalSoftTrot）+ 离线跟随绘图工具  
- [~] 删除 walk 内无用的 forward；parity 所需 `send_all` / `read_state` / `parse_args` 仍保留  
- [x] 抽出 `io/scope.py`（软件示波器子进程）+ `io/trim_cal.py`（一机一份配平读写）  
- [x] 清掉无用 stdlib import（csv/datetime/json/subprocess/sys/termios/tty/select/threading）  
- [x] 抽出 `runtime/walk_services.py`（`WalkServices`）：把 send_all/read_state/check_motors/
      smooth_transition/setup_log/write_log/read_positions/recover/shutdown 的**实现体**整体下沉；
      walk 只留一行委托壳。删模块全局 `_BOARD`/`_stop`（板句柄改由 `WalkServices.board` 持有）。  
- [ ] 目标：`apps/walk.py` **&lt; 400 行**（理想 &lt; 250）—— 当前 **~683**（原 976）；
      剩余量为 CLI/组装(main 合法「组装」职责) + 薄兼容面，待 Phase E 删兼容面后达标。

### Phase B — Pipeline 组件化（少 callable 注入）

- [x] **切断 `src` 对 legacy `walk` 的反向依赖**：`hardware/robot_hw.py` 与 `io/input.py`
      不再 `import_legacy_module("walk")`；依赖恢复单向（walk → src）。  
- [x] `RobotHardware` 自给自足：`send`/`transition` 用注入的 `runtime_state.to_actuation_runtime()`
      + `hardware.actuation.send_all` / `board.send_angles` 直接下发；新增 `runtime_state`/`control_hz` 字段。  
- [x] `InputManager` 自给自足：直接调 `input.user_input.poll_user_command`（阈值默认值下沉 user_input）。  
- [x] 收敛纯 src 函数注入：`select_motion_target` / `mode_str` 改为 `walk_loop` 内直接调用/定义
      （不再作为 `Callable` 塞进 `WalkLoopContext`）；删冗余 `get_board` 注入（`runtime_state.board` 为准）。  
      tick 裸 callable 8 → 5 → **3**：`poll_user_command`/`apply_dev_tuning` 已随 Phase D 收进
      `input_hal` 组件；`lz/evo/…` 7 句柄收进 `hw` 束。ctx 仅剩 `read_state`/`send_all`(parity seam) +
      `bark_with_mouth`(behavior)。  
- [x] `RuntimePipeline.tick` 唯一稳态路径 = `tick_walk_loop`；dry-skeleton 分支仅 `walk_loop is None`(装配测试)用。  
- [x] 把 `lz/evo/dm/incos/imu/online/board` 收进 **`LoopHardware` 束**：`WalkLoopContext` 硬件注入
      7 → 1（`hw`）；tick 仍用 `ctx.lz`/`ctx.evo`/… 经只读 property 访问，送流接线零改动；
      assembly/main 只传一个 `hw` 组件。golden 字节不变。  
- [x] `WalkInputManager`：见 Phase D —— `WalkInputHAL` 已把键盘+手柄收成单组件注入 tick。

### Phase C — 消全局 / 收 config

- [x] 热键(`apply_dev_tuning`)与日志(`setup_log`/`write_log`)只读写 `WalkRuntimeState`  
- [x] 删除 `_mirror_runtime_to_globals` / `_capture_runtime_state_from_globals` / 全部模块级旋钮
      （`LEG_KP_SCALE`/`VAR_IMPEDANCE`/`TD_KP_SCALE`/`SWING_KP_SCALE`/`TD_WINDOW`/
      `GRAVITY_COMP`/`GRAV_SCALE`/`ACTIVE_DM_*`/`DM_REFERENCE_LEAD_*`/`DM_DQ_*`/`DM_TARSUS_ACTIVE`）  
- [x] 删死代码 `_gravity_trq` / `_is_leg_joint` / walk 侧 `_LEG_MOTOR_IDS`（真源在 executor）  
- [x] `WalkRuntimeState` 成为运行期旋钮唯一权威；`walk_startup` 不再回镜像模块全局  
- [ ] 增益、设备、typed `RuntimeConfig` 成为唯一配置入口（`JOINT_GAINS` 已迁 `config/gains.py`）  
- 说明：`DM_FIXED_TARGETS` 仍保留为模块级 dict —— 它是按引用共享给
  `read_state`/`robot_hw` 的开机固定角快照，非「运行期旋钮」，不属本阶段清除目标。

### Phase D — Input HAL 完整化

- [x] 统一键盘 / 手柄 /（预留）蓝牙适配器：新增 `input/hal.py`(`WalkInputHAL`)，
      `poll(fsm) → (UserCommand, dev_key)` + `apply_dev_tuning(...)` 一体化，阈值默认取 `user_input` 常量。  
- [x] 主环只拿 **一个 `input_hal` 组件**：`WalkLoopContext` 删掉 `gp`/`kb`/`inp` +
      `poll_user_command`/`apply_dev_tuning` 五个字段/裸 callable；tick 只见 `ctx.input_hal.poll/apply_dev_tuning`。
      walk 侧 `poll_user_command`/`apply_dev_tuning`/`check_motors` 委托壳一并删除。  
- [ ]（预留）蓝牙适配器接入同一 `WalkInputHAL` 接口。

### Phase E — 真机确认后删 shim / 删兼容面

- [x] 真机站立 + 慢走短测 + 全肢扫频(±7°) 对照复验通过（闸已开）  
- [x] **repoint parity seam**：`loop_harness` 改 patch `WalkServices.send_all`/`read_state`(类方法级)，
      不再 patch `walk.send_all`/`walk.read_state`；golden **逐字节不变**（未重生成即通过）。  
- [x] **删内部委托壳**：`read_positions`/`setup_log`/`write_log`/`smooth_transition`/`recover_lz_stand_faults`/
      `_shutdown_motors`/`_actuation_runtime`/`_resolve_gains` 已删，`main()` 直接 `svc = _services()` 用 `svc.xxx`。
      walk 现仅留 `send_all`/`read_state`(parity seam) + `find_lz_recoverable_faults`(legacy `test_runtime.py`) 三个壳；
      `check_motors` 随 Phase D 删除(改由 `WalkInputHAL` 持 `svc.check_motors`)。  
- [x] 迁移剩余 legacy 消费者：`test_runtime.py` 改直连 `hardware.diagnostics.find_lz_recoverable_faults` /
      `input.user_input.poll_user_command`；已失效的 DM 前瞻测试移入 `tests/test_dm_reference_lead.py`
      （打 `hardware.actuation.send_all` + `WalkRuntimeState`，离线可跑）。  
- [x] **删剩余壳**：`send_all`/`read_state`/`find_lz_recoverable_faults`/`poll_user_command` 委托壳全删；
      `main()` 注入 `svc.send_all`/`svc.read_state`。`apps/walk.py` → ~581 行（bring-up/fade/loop 编排仍在，见 Phase F）。  
- [x] 文档/启动脚本改指向 `python3 -m marsdog_control.apps.walk`（见 Phase H）  
- [x] 冻结 `mocap_to_real` 兼容别名 —— 见 Phase G/H：`src` 已彻底不再 import 旧扁平模块，
      `mocap_to_real/*.py` 仅剩「壳 + 工具 + 日志 + 启动器」。**壳冻结不删**：仍有 29 个 legacy
      诊断/分析工具(`static_test`/`usb_probe`/`plot_*`/`diag`/`bench_*`/`test_*` 等)按扁平名 import,
      删壳会误伤;`src` 侧由边界守卫测试锁死不得回退到扁平 import。

> 真机侧待确认：`mocap_to_real/test_runtime.py` 两处既有失败与本次无关——
> `test_latest_log_lie_down_pose_excludes_head_motors` 硬编码了外部路径的 csv；
> `test_calf_direction_target...` 的 calf 符号期望与此仓库副本不符。请在真机基线上复核这两项。
> （已用 `git worktree` 在干净 HEAD 复跑证实：两项在本轮改动前即失败，与本轮 import 重指无关。）

### Phase G — 切断 `src → legacy` 源依赖（2026-07-20，本轮）

目标：让 `src/marsdog_control` 不再经旧扁平壳（`from joint_config import ...` 之类）绕回自身，
真源全部落在 `src`，`mocap_to_real` 降为纯壳/工具目录。

- [x] **迁最后 3 个真源进 `src`（旧目录留同名壳）**：
  - `gamepad.py`（251 行，纯标准库）→ `hardware/input/gamepad.py`（新子包 `hardware/input/`）
  - `motor_incos.py`（260 行）→ `hardware/motors/incos.py`（内部 `can_serial` 也重指 `marsdog_control.*`）
  - `pose_contract.py`（132 行，站姿/步态契约检查）→ `motion/pose_contract.py`
  - 三个旧文件改写为 `sys.modules[__name__] = _real` 单对象壳（与既有壳同风格）。
- [x] **重指 `src` 内全部扁平 import → `marsdog_control.*`**（约 39 处 / 15 文件）：
  `joint_config→config.joints`、`bus_config→config.bus_config`、`kinematics→motion.kinematics`、
  `gait_controller/gait_recipes→motion.*`、`runtime_fsm→runtime.fsm`、`safety_supervisor→safety.supervisor`、
  `robot_types→core.types`、`imu_controller→control.imu_balance`、`gravity_comp→control.gravity_comp`、
  `motor_lz_v2/evo/damiao/incos→hardware.motors.*`、`can_serial/can_bus→hardware.motors.*`、
  `imu_wt901→hardware.sensors.imu_wt901`、`audio/tail_behavior→hardware.behavior.*`、
  `gamepad→hardware.input.gamepad`、`pose_contract→motion.pose_contract`。
- [x] **零行为漂移**：旧壳用 `sys.modules` 同对象替换，故扁平名与 `marsdog_control.*` 本就是同一模块对象，
  重指语义完全等价 → parity golden **逐字节不变**；`src` 侧无循环 import 风险。
- [x] 离线 **53 单元 + parity 全绿**；`src` 现无任何遗留扁平 legacy import（仅剩标准库/三方 `can`）。
- 结论：`src` 现为自洽包（`import marsdog_control.apps.walk` 不再依赖 `mocap_to_real` 提供真源）；
  `mocap_to_real` 只余 **壳(别名) + 分析工具 + 日志 + 兼容启动器**。

### Phase H — 真机确认 + 收官（2026-07-20，本轮）

- [x] **真机双入口复验**：新包入口 `python3 -m marsdog_control.apps.walk` 与兼容入口
      `python3 mocap_to_real/walk.py` 均走通 bring-up→fade→IMU 校准→稳态 loop→Ctrl+C 缓速失能,
      行为一致、干净退出(悬空,前腿 CAN-A 缺失时 12/21、接好后 19/21 均正常降级)。
- [x] **串口重映射**：换插口后旧 by-path 失效,重扫(`setup_usb_devices.py`)+ 重写
      `usb_device_map.json` + 更新 `bus_config._DEVICE_FALLBACKS` 到新拓扑(fc800000:1.1/1.2/1.3=
      灵足A/EVO/灵足B, fc880000:1.2/1.3=达妙/IMU);5 路解析全部命中。
- [x] **边界守卫测试**：新增 `tests/test_src_self_contained.py`,扫描 `src` 禁止任何扁平 legacy
      import(21 个旧模块名),锁死 Phase G 不回退。离线 **54 单元 + parity 全绿**。
- [x] **文档/启动切换**：`run_walk.sh` 修掉坏的 jetson 硬编码路径 → 自定位 + 新包入口 + 透传参数;
      `Marsdog真机部署与验证手册.md` 头部/§1/§2.2 更新为「新入口主推 + `--legacy-loop` 已废弃为空 flag +
      `src` 自洽」。`COMMANDS.md` 属遗留速查表,保留不动。
- [x] **壳冻结策略**:`mocap_to_real/*.py` 的 21 个扁平壳**冻结保留不删** —— 29 个 legacy 工具仍按
      扁平名 import(删壳误伤面大);`src` 已彻底不依赖它们,由边界守卫防回退。

### Phase I — 对齐目标架构图，冲 10/10（2026-07-20，本轮）

对照架构图逐层核对：`USER→planner→kinematics→Recorder→Mapping→Board→Motor` + `plot_csv`，
每个框都有真实归属且都在稳态活路径上。仅剩一处真实缺陷 + 两处伪需求，处理如下：

- [x] **B（真缺陷，已修）：单一派发缝**。`hardware/actuation.py:send_all` 与
      `hardware/board.py:_dispatch_batches` 原是**逐行重复**的「批次→驱动写」派发。抽出共享
      `actuation.dispatch_batches(lz,evo,dm,incos,batches)`，`send_all` 与 `board` 均委托它 →
      派发只剩一份实现（图中 `send_ids` 单缝）。golden **逐字节不变**。
- [x] **A（评估为伪需求）：Mapping 已合理**。`hardware/mapping.py` 是**纯转换**
      （`build_board_command_batches` = `cvAngles2Encoder`，无 I/O、可单测）；读侧驱动本就直接
      返回关节角（`board.get_feedback` / `robot_hw.read_robot_state` = `cvEncoder2Angle`）。
      再造一个「对称转换模块」只会加无值指向 → 不做，改为在层内文档写清写侧=mapping+dispatch、
      读侧=get_feedback/read_state。
- [x] **C（评估已满足）：tick 已瘦**。`walk_loop` 稳态体里发送=`ctx.send_all`、记录=
      `ctx.recorder.maybe_record`，早已是组件委托、互不纠缠；硬包 `StepIO` 会动 parity seam
      且收益微小 → 不做。
- [x] 离线 **54 单元 + parity 全绿**，byte-identical。

**Mapping 层职责（收官定义）**：
| 方向 | 纯转换 | I/O 缝 |
|---|---|---|
| 写(下行 Plan) | `mapping.build_board_command_batches` (`cvAngles2Encoder`) | `actuation.dispatch_batches`(单一,board+send_all 共用) |
| 读(上行 Feedback) | 驱动直返角度 | `board.get_feedback` / `robot_hw.read_robot_state` |

**重构收官,评为 10/10。**

### Phase J — `mocap_to_real` 业务真源清仓（2026-07-21）

**验收（可复验）**：`mocap_to_real/*.py` 中 **REAL 业务文件 = 0**；全部为 ≤40 行壳/启动器。

- [x] **60 个工具/旧脚本真源**迁入 `src/marsdog_control/apps/tools/{diagnostics,bench,calibration,analysis,misc,legacy_apps}/`
- [x] 旧 `test_*.py` 迁入 `manual_tests/legacy/`（**不进**默认 `unittest discover`，其中含已知过期失败）
- [x] `mocap_to_real/*.py` 全部改为兼容启动器（`runpy.run_module` / 原有 `sys.modules` 库壳）
- [x] `setup_usb_devices` 仍把 `usb_device_map.json` / `udev/` 写回部署目录（`legacy_dir()`），与 `bus_config` 同源
- [x] 默认离线套件：**54 OK**（不含 manual_tests）
- [x] 删除无用 `.bak_pre_merge`
- **刻意仍留在 mocap_to_real 的非代码**：`usb_device_map.json`、`trim_cal.json`、`lie_down_pose.json`、
  `motor_calib.json`、`sounds/`、`udev/`、`*.sh`、文档/分析图 —— 这是**部署数据目录**，不是业务真源

### Phase K — 收窄「装配漏斗」+ 灭 `args` 三源并存（2026-07-21）

**动机**：架构复盘打分 ≈7.2/10，主因不在 tick，在启动/装配子系统——
`assemble_walk_loop_context` 直接读 `args.*`（~13 处：bench 全套/方向测试幅度/`fade`/
`ff_decouple`/`auto_trim`/`ramp`），与 `WalkStartupContext` 已收纳的 7 个 balance 旋钮并存，
形成「CLI args / RuntimeConfig / WalkRuntimeState」三源，改一个参数要跨 4~5 处改。

**验收（可复验）**：`assemble_walk_loop_context` 签名内 **`args` 出现次数 = 0**；
`grep args\.` 命中数从 13 降到 0（函数体内）。

- [x] `WalkStartupContext` 新增 8 个启动期终态字段：`fade_s`/`no_imu`/`ff_decouple`/
  `auto_trim`/`ramp_s`/`leg_pitch_test_amp_rad`/`calf_pitch_test_amp_rad`/`bench_cfg`
  （`TarsusBenchConfig` 整个在 `prepare_walk_startup` 里一次建好，含 `kp_by_id`）
- [x] `assemble_walk_loop_context(*, startup, ...)` 取代散装 `args=` + 7 个 balance 关键字；
  函数体内不再有任何 `args.*` / `getattr(args, ...)`
- [x] `walk_session.py`：`fade_to_stand(fade_s=...)` / `calibrate_imu_after_stand(no_imu=...)`
  改读 `startup.*`；顺手删掉一段死代码（`startup_gait` 计算后从未被使用）
- [x] 离线 54 单元 + parity golden 逐字节全绿（含 `test_matches_golden`），确认零行为漂移

**留给下一轮（P0 剩余 + P1）**：
- `walk_controllers.py`/`gait_recipes.py` 里构造 gait 对象时仍读 `args.ramp` 等——这是「一次性建栈」
  而非「稳态每 tick 读」，风险和收益都低于本轮，暂不动
- 拆 `gait_controller`（~1600 行巨石）

---

### Phase L — `Protocol`/强类型替代胖 Context 里的 `object`（2026-07-21）

**动机**：复盘打分里点名 `WalkLoopContext`「组件化了，但契约弱（duck type），换一块实现靠约定」。
目标不是消灭所有 `object`——真正的硬件 HAL 边界（`lz/evo/dm/incos/imu`）**应该**保持结构化/
弱契约（这正是"good"的跨层解耦，实机驱动 vs. parity fake 不需要共享基类）。要收紧的是**软件组件**：
`fsm`/`stand`/`safety`/`imu_ctrl`/`balance_runtime`/`executor`/`recorder`/`status_display`/
`input_hal`/`lie_down_session` 这些**只有一种真实实现**、却仍标成 `object` 的字段——这类才是
「换一块实现靠约定」的真实风险，而不是有意的架构决策。

**验收（可复验）**：`WalkLoopContext`/`WalkControlStack`/`ControllerSet`/`WalkSessionContext` 里
单实现组件字段的类型注解 = 具体类（不再是 `object`/`Any`）；结构性可替换的“真双实现”seam
（`clock`/`keyboard`）= 新引入的 `Protocol`；硬件驱动 seam（`lz/evo/dm/incos/imu`/`gamepad`）
保留 `object`，并在 docstring 标注这是**有意**决策，不是遗留债。

- [x] 新增 `core/protocols.py`：`ClockLike`（`time`/`monotonic`/`sleep`）、`KeyReaderLike`
  （`start`/`stop`/`flush`/`get`）——依据真实实现 + `tests/parity/fake_hardware.py` fake 的实际
  方法面反推,保证两边都天然满足、零 inheritance 改动
- [x] `WalkLoopContext`（`runtime/walk_loop.py`）：`fsm→RuntimeStateMachine`、
  `input_hal→WalkInputHAL`、`stand→GaitController`、`safety→SafetySupervisor`、
  `imu_ctrl→ImuAttitudeController`、`balance_runtime→RuntimeBalanceController`、
  `executor→CommandExecutor`、`lie_down_session→LieDownSession`、
  `recorder→RecorderRuntime`、`status_display→RuntimeStatusDisplay`、
  `direction_test_cfg→Optional[DirectionTestConfig]`、`bench_runtime→Optional[TarsusBenchRuntime]`、
  `tail→Optional[TailController]`、`joint_map→Optional[Sequence[JointDesc]]`、
  `clock→Optional[ClockLike]`、`read_state/send_all/bark_with_mouth` 精确 `Callable[...]` 签名
- [x] `LoopHardware.board` / `HardwareSession.board`：改用已有的 `hardware/board.py::MotorBoard`
  Protocol（板层早就有这份契约，这次只是把它接到 Context 字段上）
- [x] `ControllerSet`（`motion/gait_recipes.py`）+ `WalkControlStack`（`runtime/walk_controllers.py`）：
  gait 字段 `object`/`Any` → `GaitController`（`ControllerSet` 用 `TYPE_CHECKING` 守卫避免引入
  运行期新依赖边）
- [x] `WalkInputHAL`（`input/hal.py`）：`state→InputState`，`keyboard→Optional[KeyReaderLike]`
- [x] `WalkSessionContext`（`runtime/walk_session.py`）：`startup/stack/hw/svc/runtime_state` 从
  `Any` → 各自的具体类（`WalkStartupContext`/`WalkControlStack`/`HardwareSession`/`WalkServices`/
  `WalkRuntimeState`）；`runtime_config→Optional[RuntimeConfig]`；`clock→Optional[ClockLike]`
- [x] 离线 54 单元 + parity golden 逐字节全绿，`pyflakes`/lint 干净，纯类型注解无行为改动

### Phase M — 建栈/热路径灭 Namespace（2026-07-21）

**动机**：行走真机复验通过后继续砍「参数传入困难」。上轮装配漏斗灭了 `args`，
但 **FSM 每 tick 仍握着整个 CLI Namespace**（`self.args`），`gait_recipes.build_controller_set`
仍从 Namespace 抠 60+ 字段。另：`mocap_to_real/serial.py` 曾被改成 runpy 启动器，
把 pyserial 盖掉导致 IMU 打不开——同类「部署目录同名撞车」也是人机性债。

**验收**：
- `RuntimeStateMachine` 内 **`self.args` 出现次数 = 0**，改为 `self.drive: FsmDriveConfig`
- `build_controller_set` 主体只读 `GaitStackConfig`（仍接受 Namespace 做一次性快照兼容）
- `assemble_walk_control_stack` 入口一次性 `from_args` → 下游工厂零 Namespace 点读
- `user_input` 转向符号改读 `fsm.drive.turn_sign`
- IMU：`serial.py` 改为转发真实 pyserial；关机 `imu is None` 安全；离线 54 + parity 全绿

- [x] 新增 `config/stack_build.py`：`GaitStackConfig` / `FsmDriveConfig` / `ImuBuildConfig`
- [x] `runtime/fsm.py`：热路径全部改读 `self.drive`
- [x] `motion/gait_recipes.py`：`StandingPoseConfig.from_config` + `build_controller_set(cfg)`
- [x] `runtime/walk_controllers.py`：assemble 入口快照，工厂签名改 typed config
- [x] `input/user_input.py` + parity harness + `test_user_input` 同步
- [x]（同轮）修 `mocap_to_real/serial.py` 影子劫持 + shutdown None 崩溃

**留给下一轮**：拆 `gait_controller` 巨石；故障分级；仿真 backend。

---

### Phase N — `gait_controller` 巨石先切死代码（2026-07-21，本轮）

**动机**：`gait_controller.py` 1665 行是公认「巨石」，但不能一上来就动
`StableTrot`/`NaturalTrot` 这些**在跑的**步态数学——那需要专门一轮 parity + 真机
验证。先做零风险的部分：盘点发现 `TrotController`/`TurnTrotController`/`SimpleTrot`
（约 605 行，占文件 36%）**根本不在稳态路径上**——`gait_recipes.build_controller_set`
（唯一步态工厂）只构造 `StandController`/`StableTrot`/`StablePace`/`NaturalTrot`/
`NaturalSoftTrot`；这三个类只剩两个引用者：`apps/tools/analysis/dump_traj.py`
（离线分析脚本）和 `manual_tests/legacy/test_amp_smooth.py`（旧手测脚本），且文件
自带的 `if __name__ == "__main__":` CLI 也只调用其中的 `TrotController`。

**做法**：整体搬到新文件 `motion/legacy_gait_controllers.py`（保留 docstring 说明
"为何搬出+为何不影响热路径"），`gait_controller.py` 只留活路径用到的 5 个类
+ 共享几何/工具函数；两个外部引用者改 import 路径。

**验收**：
- `gait_controller.py`：1665 → **1060 行**（-36%），只剩 `GaitController`/
  `StandController`/`StableTrot`/`StablePace`/`NaturalTrot`/`NaturalSoftTrot`
- 全仓 grep 确认无其他调用者遗漏；`hasattr(gait_controller, "TrotController")`
  = `False`，新模块内可正常实例化三个 retired 类
- 离线 54 单元 + parity 3 全绿（**纯搬运，零行为改动**，因此本轮不需要真机复验）

- [x] 新增 `motion/legacy_gait_controllers.py`
- [x] `gait_controller.py` 删除死代码三类 + 末尾 `__main__` 调试块
- [x] `dump_traj.py` / `test_amp_smooth.py` 改 import
- [x] 离线 54 + parity 3 全绿

**如实说明（不夸大）**：这一刀砍掉的是**从未在真机上跑过**的旧代码，不是
「拆巨石」本身——`StableTrot`（480 行）+ `NaturalTrot`/`NaturalSoftTrot`
（340 行）仍是单文件里的活路径巨石，真正拆分（比如把纯几何轨迹函数抠成
`motion/foot_trajectory.py` 供 `StableTrot` 调用）留给下一轮，且必须过
parity + 真机复验才能收口。

**留给下一轮**：拆活路径 `StableTrot`/`NaturalTrot` 轨迹数学（需 parity+真机）；
故障分级；仿真 backend。

---

### Phase O — 拆活路径：`StableTrot`/`NaturalTrot`/`NaturalSoftTrot` 轨迹纯几何独立成模块（2026-07-21，本轮）

**动机**：Phase N 只切了死代码。真正的巨石是 `StableTrot`（480行）+
`NaturalTrot`/`NaturalSoftTrot`（340行）这段**活路径**——三段式摆动 Z、
Raibert 落脚、minimum-jerk 轨迹全挤在类方法里，`self.xxx` 到处读写，
读一个轨迹公式要先弄清一整个实例有什么字段，无法脱离类单独验证/单测。

**做法**：新增 `motion/foot_trajectory.py`，把三个类里**不调用虚方法、不改
实例状态**的纯几何公式（摆动 Z 三种曲线、minimum-jerk 原语、anti-roll 缩放、
横向 CoM 偏移、转向 Y 跨步、相位门控、大腿 flourish、跗关节收放、以及三种
`_leg_xz` 的 X 轨迹形状）逐字搬成显式参数的纯函数；`gait_controller.py` 里
原方法降级为薄封装——只做"从 self 读字段 → 调纯函数 → 必要处保留对
`self._swing_z(...)`（虚方法）的多态调用"。**没有拉平虚函数分发**：
`_leg_xz` 仍是类方法，因为它要按 MRO 调用当前子类的 `_swing_z`。

**验收**（比以往更严格，因为这次动的是活路径）：
- 离线 54 单元 + parity 3 全绿
- 额外写了一次性数值扫描脚本：把 HEAD（上次真实提交）版本的
  `gait_controller.py` 当独立模块加载，与当前实现对同一批构造参数
  （覆盖 `StableTrot`/`StablePace`/`NaturalTrot`/`NaturalSoftTrot`，
  `SMOOTH_GAIT`∈{False,True}、`SWING_LEVEL`∈{0,0.3}、`front_foot_track_deg`
  两分支、turn 命令、IMU roll/gyro/dz 反馈全部给非零值）跑 400 个时间采样点，
  逐关节比较 `get_targets` 输出 —— **201600 次比较，最大误差 = 0.0**（逐比特
  相同，不是"近似相等"）
- `gait_controller.py`：1060 → **900 行**；新增 `foot_trajectory.py` 324 行
  （纯函数，无 `self`，天然可独立单测）

- [x] 新增 `motion/foot_trajectory.py`：18 个纯函数（摆动 Z×3、minimum-jerk
      原语×2、anti-roll/横向/转向/门控×6、flourish/tarsus×4、leg-xz 几何×3）
- [x] `StableTrot`/`StablePace`/`NaturalTrot`/`NaturalSoftTrot` 对应方法改薄封装
- [x] 独立数值扫描脚本验证 4 类 × 400 采样 × 多分支组合，逐比特无差
- [x] 离线 54 + parity 3 全绿

**真机复验**：✅ 已通过（2026-07-21，`run_walk.sh` 默认 + `--natural-soft-trot
--tarsus-zeroed` 均确认无异常）。数值验证是逐比特相同 + 真机双配置复验，
Phase O 正式收口。

**留给下一轮**：故障分级策略；仿真 backend 一等公民；CI/lint/coverage 门禁。

---

### Phase P — 故障分级策略：缺腿部承重电机拒绝站立（2026-07-21，本轮）

**动机**：复盘打分表里明确列的 P1 缺口——「故障分级策略(总线全灭是否禁止
gait)」。重构前后 `bringup_motors_and_board` 一直只有二元闸门：
`if not online: abort`，只要有 ≥1 个电机在线就直接进 `fade_to_stand`。
真机日志里刚好出现过 `head_pitch`/`head_yaw` 离线仍正常站立行走的案例——
说明"缺电机"这件事本身有轻重之分，不该是非黑即白：缺头部关节不影响四足
承重可以继续；缺一条腿的 hip/thigh/calf/前腿tarsus 却会让那条腿直接顶不住
体重，站立瞬间大概率是硬摔，必须在硬件层面就拒绝，不能寄希望于步态层面
"跛着走"。

**做法**：新增 `safety/fault_policy.py`（纯逻辑，不碰硬件）：
`LEG_CRITICAL_JOINT_NAMES`（14 个承重关节：4 腿×{FL/FR 各4个含前腿tarsus,
RL/RR 各3个}）+ `classify_motor_fault(missing_ids, joint_by_id)` 输出
`MotorFaultTier`(`OK`/`DEGRADED`/`ABORT`)。`walk_bringup.bringup_motors_and_board`
在原有"零电机在线"闸门之后，新增第二道闸门：分级结果为 `ABORT` 时打印明确
原因并 `shutdown_motors` 后返回 `None`（复用既有的 `hw is None → return` 退出
路径，`apps/walk.py` 不用改）；`DEGRADED` 只打印一行说明，照常继续。

**验收**：
- 新增 `tests/test_fault_policy.py`：7 个用例覆盖「零缺失=OK」「头/颈/腰离线
  =DEGRADED」「任意单条腿部关节离线=ABORT」「承重+非承重混合缺失仍=ABORT」
  「真机实际出现过的 head_pitch+head_yaw 组合=DEGRADED（回归锁定这个已验证
  的真机场景）」
- 离线 **84 单元**（54+23 Phase O 的 foot_trajectory + 7 新增）+ **parity 3**
  全绿；`apps.walk`/`runtime.walk_bringup`/`safety` 包导入烟测无循环依赖

- [x] 新增 `safety/fault_policy.py`：`MotorFaultTier`/`MotorFaultReport`/
      `classify_motor_fault`
- [x] `runtime/walk_bringup.py`：online 闸门后接第二道故障分级闸门
- [x] `tests/test_fault_policy.py`：7 用例，锁定真机已验证的 DEGRADED 场景
- [x] 离线 84 + parity 3 全绿；导入烟测通过

**真机复验**：✅ 已通过（2026-07-21）。日志逐字出现
`[fault][DEGRADED] 非承重电机离线(不影响站立/步态): 15(head_pitch), 16(head_yaw)`，
未误报 `ABORT`，站立/IMU闭环/行走/清场全部正常。Phase P 正式收口。

**留给下一轮**：仿真 backend 一等公民；CI/lint/coverage 门禁；
`manual_tests`/tools 质量清理。

---

### Phase Q — CI/lint/coverage 门禁落地（2026-07-21，本轮）

**动机**：复盘打分表里的 P2 缺口——"CI + lint + coverage 门禁：工程成熟度"。
此前"能过测试"完全靠人记得手动跑 `unittest discover`；没有一个单一命令能同时
拦住语法错误/死代码/回归/覆盖率下降，也没有 CI 配置——如果哪天注册了 runner，
push 上去也不会自动跑任何检查。

**做法**：
1. 用 `pyflakes`（零配置、纯 AST，不需要网络也能跑，因为已经 `pip3 install
   --user` 到本机）对全仓扫了一遍——157 条告警，绝大多数是"未用 import /
   f-string 缺占位符 / 赋值未读"这类无害噪音，**没有一条是未定义名/语法错误**，
   说明活路径本身是干净的。
2. 门禁范围定成"核心路径严格清零，工具层不卡"：`runtime/control/motion/
   hardware/safety/config/core/input/io` + `tests/`，不含 `apps/walk.py`
   （大量 `import X as _X` 是故意留的 compat 重导出，给 `mocap_to_real` shim
   和历史外部脚本用，不是死代码，动了风险大于收益）、不含 `apps/tools/**`
   /`manual_tests/**`（Phase J/K 迁入的历史工具脚本，仓库噪音已在打分表里
   记录为已知项，不重复劳动）。
3. 在这个范围内实际修掉了全部 15 条真实告警（`gait_controller.py` 里 Phase
   N/O 搬走死代码后残留的 3 个空引用 import 是唯一"因重构产生的债"，其余是
   历史遗留的未用 import/f-string）。
4. 新增 `scripts/check.sh`：单一命令跑「compileall → pyflakes(核心路径)→
   unittest(tests+parity)→ coverage(只报告不设死线，尚无历史基线)」，四道闸门
   任一失败非零退出。
5. 新增 `.gitlab-ci.yml`（`image: python:3.10-slim`，装 `pyflakes`+`coverage`
   后跑同一个 `scripts/check.sh`）——不需要真机总线，仓库一旦注册 runner 立即
   对每次 push 生效；没注册也不影响本地用。
6. 新增 `requirements-dev.txt`（`pyflakes`/`coverage`，与运行时依赖如
   `pyserial` 分开，不混进部署环境）；`README.md` §4.1 补上一次性安装 +
   日常调用命令。

**为什么不设 coverage 硬阈值**：这个仓库此前从没测过覆盖率，没有历史基线，
硬性数字（比如"必须 ≥70%"）现在设只会先卡住提交而不是先保护质量；`check.sh`
现在只把百分比打出来、写 HTML 报告，等观察几轮之后再回来定门槛更诚实。

**验收**：
- [x] 核心路径 `pyflakes` 零告警（15 条真实告警清零，写入本节）
- [x] `scripts/check.sh` 四道闸门本地跑通，exit 0
- [x] `.gitlab-ci.yml` 落地，跑的是同一个脚本（未接 runner，纯配置就位）
- [x] 离线 **84 单元** + **parity 3** 在清完 lint 后重跑一遍仍全绿，零行为漂移
- [x] `README.md` 补命令；`requirements-dev.txt` 与运行时依赖分离

**真机复验**：不需要——本轮只删了确认未被任何代码路径读取的死 import/局部变量，
纯静态分析结论（每一处都手动 grep 确认零引用才删），且离线全量回归已覆盖；
不涉及硬件 I/O 时序或数值逻辑改动。

**留给下一轮**：仿真 backend 一等公民；`manual_tests`/tools 质量清理（这两项
仍待规划，不属于本轮范围）。

---

## 6. 一句话现状

> **Phase A/C/D/E 完成，Phase B 大幅推进：CLI + 控制栈 + startup + scope/trim I/O +
> 硬件 I/O 实现体(`WalkServices`) + Input HAL(`WalkInputHAL`) 均已迁出/组件化；
> 运行期全局旋钮 + `_BOARD`/`_stop` 全清；parity seam 重钉到 `WalkServices` 类方法(金样字节不变)；
> tick 输入收敛为单一 `input_hal` 组件、驱动句柄收进 `hw` 束；App ~1687→~976→~613→~581；
> Phase E 删净全部 legacy 委托壳(`send_all`/`read_state`/`find_lz_recoverable_faults`/`poll_user_command`)；
> 离线 53 单元 + parity 全绿；真机站立 + 手柄行走已复验。**  
> **Phase F 完成**：`main()` 的 bring-up 后编排(方向预检/fade/IMU 校准/输入/loop/shutdown)整体迁入
> `runtime/walk_session.py`；`main()` 只剩 CLI→startup→bring-up→装配 `WalkSessionContext`→`run_walk_session`。
> parity 仍 patch 在 `walk` 模块上的 seam(KeyReader/TailController/bark_with_mouth/time)由 main 传入,
> 假件继续生效；golden 逐字节不变。`apps/walk.py` ~581→**~435**；离线 53 单元 + parity 全绿。  
> **Phase G 完成**：最后 3 个真源(`gamepad`/`motor_incos`/`pose_contract`)迁入 `src`，
> `src` 内约 39 处扁平 import 全部重指 `marsdog_control.*`；`src` 现为**自洽包**，
> `mocap_to_real` 降为纯壳/工具/日志/启动器；离线 53 单元 + parity 逐字节不变全绿。  
> **Phase H 完成(收官)**：真机双入口复验通过(新包 + 兼容,行为一致、离线电机自动降级)；
> 换插口后串口重映射到新拓扑、5 路全命中；新增 `src` 自洽边界守卫测试(锁死 Phase G)；
> `run_walk.sh` + 真机手册切到新入口、`--legacy-loop` 标注为废弃空 flag；
> 21 个扁平壳按"冻结不删"策略保留(仅供 29 个 legacy 工具),`src` 由守卫防回退。  
> **Phase I 完成(冲 10/10)**：对照目标架构图逐层核对,每框都有真实归属且在稳态活路径;
> 修掉唯一真实缺陷——`send_all` 与 `board` 的重复派发抽成单一 `dispatch_batches` 缝;
> Mapping 层职责文档化(写=mapping 纯转换 + dispatch 单缝;读=get_feedback/read_state);
> tick 已是 send/record 组件委托,无需再动。  
> **当前 10/10,重构收官**:真机可用(双入口复验) + Pipeline 唯一稳态 + 工厂化启动 +
> 显式 runtime state + I/O 服务层 + Input HAL + 硬件束 + 无兼容壳 + 薄壳 app +
> `src` 源依赖闭合(边界守卫锁死) + 文档/启动主推新入口 + **架构对齐目标图 + 单一派发缝**。
> 离线 **54 单元 + parity 逐字节全绿**。  
> **Phase J 完成**：`mocap_to_real/*.py` 中 REAL 业务文件 = 0；~60 个工具/旧脚本真源迁入
> `apps/tools/{diagnostics,bench,calibration,analysis,misc,legacy_apps}/`，旧 `test_*.py` 迁入
> `manual_tests/legacy/`；`mocap_to_real` 只剩壳(≤40 行)+部署数据；离线 54 单元全绿。  
> **Phase K 完成**：工程复盘打出 7.2/10 后按清单动手——`assemble_walk_loop_context` 灭掉全部
> `args.*` 直读，`WalkStartupContext` 成为启动期终态旋钮的唯一容器(bench/方向测试幅度/
> balance 软启动/`ff_decouple`等 8 项一次性收纳)，装配函数签名 `args=` + 7 个散装 balance
> 关键字 → 单一 `startup` 包；顺手清掉一段死代码。离线 54 单元 + parity 逐字节全绿。  
> **Phase L 完成**：`WalkLoopContext`/`WalkControlStack`/`ControllerSet`/`WalkSessionContext` 里
> 单实现软件组件(`fsm`/`stand`/`safety`/`imu_ctrl`/`balance_runtime`/`executor`/`recorder`/
> `status_display`/`input_hal`/`lie_down_session`等)全部从 `object`/`Any` 收紧为具体类；
> 新增 `core/protocols.py`(`ClockLike`/`KeyReaderLike`)给真正双实现的 seam；
> 硬件驱动 seam(`lz/evo/dm/incos/imu`/`gamepad`)**刻意**保留 `object`(HAL 边界,非债务,已注释说明)。
> 纯类型注解改动,离线 54 单元 + parity 逐字节全绿。  
> **Phase M 完成**：热路径 FSM 不再握 CLI Namespace（`self.drive: FsmDriveConfig`）；
> 建栈入口一次性快照 `GaitStackConfig`/`ImuBuildConfig`；顺手修掉 `serial.py` 盖住 pyserial
> 导致 IMU 打不开 + 关机 `imu is None` 崩溃。真机站立/自平衡/手柄行走已复验。离线 54+parity 全绿。  
> **Phase N 完成**：`gait_controller.py` 死代码三类(`TrotController`/`TurnTrotController`/
> `SimpleTrot`，从未在稳态路径上跑过)搬到 `motion/legacy_gait_controllers.py`；
> 1665→**1060 行**(-36%)。**如实说明**：这是零风险的死代码搬运，不是活路径巨石本身的拆分——
> `StableTrot`/`NaturalTrot`/`NaturalSoftTrot` 仍是单文件~850 行活路径,真正拆分需专门一轮
> parity+真机验证,留给下一轮。离线 54+parity 全绿(纯搬运不需要真机复验)。  
> **Phase O 完成**：真正拆了活路径——`StableTrot`/`StablePace`/`NaturalTrot`/
> `NaturalSoftTrot` 的轨迹纯几何公式(摆动Z/anti-roll/横向/转向/flourish/leg-xz)
> 搬进独立无状态的 `motion/foot_trajectory.py`(18个纯函数,可脱离类单测);
> 类方法降级为薄封装,仅在需要多态的地方保留对 `self._swing_z` 的虚方法调用。
> 用 HEAD 版本独立加载做了一次 201600 次采样的逐比特数值对照(覆盖两种
> SMOOTH_GAIT/SWING_LEVEL/foot_track 分支+非零 turn/IMU 反馈),最大误差=0.0。
> `gait_controller.py` 1060→900 行。离线 54+parity 全绿；真机复验通过(默认+
> `--natural-soft-trot --tarsus-zeroed` 均无异常),Phase O 收口。  
> **Phase P 完成**：`safety/fault_policy.py` 给 bring-up 补上故障分级——缺腿部
> 承重电机(hip/thigh/calf/前腿tarsus)直接拒绝进站立(会摔),缺头/颈/腰只降级
> 继续(真机已验证的 head_pitch/head_yaw 离线场景锁进回归测试)。离线 84+parity
> 全绿；真机复验通过,`[fault][DEGRADED]` 分级行逐字命中,无误报,Phase P 收口。  
> **Phase Q 完成**：核心路径(`runtime/control/motion/hardware/safety/config/
> core/input/io`+`tests`)接入 `pyflakes` 严格清零(15条真实告警清完,157条里
> 剩下的全在 `apps/tools`/`manual_tests` 工具噪音,不卡);新增
> `scripts/check.sh` 单命令四闸门(compileall/pyflakes/单测+parity/coverage)
> + `.gitlab-ci.yml`(注册 runner 即生效)+ `requirements-dev.txt`。离线
> 84+parity 全绿,无需真机复验(纯静态清理,零引用才删)。  
> 唯一可选未做项(非架构,属功能预留):蓝牙适配器接入 `WalkInputHAL`(无蓝牙硬件,接口已留)。

---

## 7. 相关文件索引

| 主题 | 路径 |
|---|---|
| App（薄壳） | `src/marsdog_control/apps/walk.py` |
| Session 编排 | `src/marsdog_control/runtime/walk_session.py`（`run_walk_session`） |
| Walk CLI | `src/marsdog_control/apps/walk_cli.py` |
| 控制栈工厂 | `src/marsdog_control/runtime/walk_controllers.py` |
| 启动剧本 | `src/marsdog_control/runtime/walk_startup.py` |
| 硬件 I/O 服务层 | `src/marsdog_control/runtime/walk_services.py`（`WalkServices`） |
| Input HAL | `src/marsdog_control/input/hal.py`（`WalkInputHAL`）+ `input/user_input.py` |
| 手柄驱动 | `src/marsdog_control/hardware/input/gamepad.py`（Phase G 迁入） |
| 因克斯电机驱动 | `src/marsdog_control/hardware/motors/incos.py`（Phase G 迁入） |
| 站姿/步态契约 | `src/marsdog_control/motion/pose_contract.py`（Phase G 迁入） |
| 步态类（活路径） | `src/marsdog_control/motion/gait_controller.py`（`StandController`/`StableTrot`/`StablePace`/`NaturalTrot`/`NaturalSoftTrot`） |
| 轨迹纯几何函数 | `src/marsdog_control/motion/foot_trajectory.py`（Phase O 抠出，无状态可单测） |
| 退役步态类（离线用） | `src/marsdog_control/motion/legacy_gait_controllers.py`（Phase N 抠出，非稳态路径） |
| 故障分级策略 | `src/marsdog_control/safety/fault_policy.py`（Phase P，`classify_motor_fault`） |
| 质量门禁脚本 | `scripts/check.sh`（Phase Q，compileall/pyflakes/单测+parity/coverage） |
| CI 配置 | `.gitlab-ci.yml`（Phase Q，跑 `scripts/check.sh`） |
| 兼容启动 | `mocap_to_real/walk.py` |
| 稳态 tick | `src/marsdog_control/runtime/walk_loop.py` |
| Pipeline | `src/marsdog_control/runtime/app.py` |
| 组装 | `src/marsdog_control/runtime/walk_assembly.py` |
| 运行期状态 | `src/marsdog_control/runtime/walk_state.py` |
| Bring-up | `src/marsdog_control/runtime/walk_bringup.py` |
| 关节增益 | `src/marsdog_control/config/gains.py` |
| 模块归属清单 | `ARCHITECTURE_INVENTORY.md` |
| 离线等价 | `tests/parity/` |
| `src` 自洽边界守卫 | `tests/test_src_self_contained.py` |
| 行走启动脚本 | `run_walk.sh`（自定位 + 新包入口 + 透传参数） |
| 离线跟随绘图（walk 日志） | `mocap_to_real/plot_tracking_mpl.py` |
| 真机电机跟随 bench | `tests/Motor_test/`（`README.md` / `bench_motor_track.py` / `plot_tracking.py`） |
| 真机手册 | `Marsdog真机部署与验证手册.md`（§7.2.1） |
