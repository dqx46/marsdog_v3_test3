# Marsdog V3 Dynamics Control Architecture (WBC+MPC)

## 概述 (Overview)
Marsdog V3 是一只高度非对称构型的四足机器人。前腿包含主动的跗关节 (tarsus，由达妙电机驱动) 或存在虚拟四连杆结构；后腿则使用了类似连杆的半被动/被动结构，跗关节通过连杆随动，并不是完全独立的驱动关节。

这种极其复杂的构型导致传统的以简化单刚体动力学 (SRB-MPC) 搭配常规全身控制 (WBC) 的开源方案（如 OCS2、MIT Mini-Cheetah）无法直接套用。为了实现对这样复杂物理系统的精确控制，我们引入 **Pinocchio** 进行全刚体动力学解算，并在其之上构建专门针对含被动关节机器人的 WBC 和 MPC 控制器。

## 构型难点与应对策略

### 1. 非对称自由度映射
- 浮动基座 (Floating Base) 拥有 6 个自由度（空间位置 XYZ + 姿态 RPY）。
- 前腿 (Front Legs)：`fl`, `fr`。关节结构为 `hip_pitch`, `thigh_roll`, `calf`, `tarsus`。
  - 主动自由度：4 个。
- 后腿 (Rear Legs)：`rl`, `rr`。关节结构为 `hip_pitch`, `thigh`, `calf`，可能带有机械联动 `tarsus`。
  - 主动自由度：3 个。
  - 被动/连杆自由度：如果在 URDF 中对后腿保留了不带电机的 `tarsus`，则该自由度是欠驱动的。

### 2. WBC 二次规划 (QP) 求解策略

全身控制器 (Whole Body Control) 的核心是解出一个所有主动关节的前馈扭矩 ($\tau_{ff}$)，以追踪期望的躯干加速度 $\ddot{x}_{des}$ 和摆动腿足端轨迹，同时满足各类物理约束。

**系统动力学方程：**
$$ M(q)\dot{v} + h(q, v) = S^T \tau + \sum J_{c,i}^T F_{c,i} $$

**非对称及被动约束处理：**
对于带被动关节或没有接线的电机对应的自由度：
- **选择矩阵 $S$**：$S$ 矩阵用于把 $n$ 维关节扭矩向量 $\tau$ 映射到总自由度上。对于 Marsdog，如果 URDF 中某关节（例如后腿的 `rr_tarsus`）是被动的，则在 $S$ 中对应的行必须严格为 $0$。
- **约束满足**：QP 求解器只能调节那些 $S$ 对角线为 1 的主动关节的 $\tau$。地面的反作用力 $F_c$ 和主动扭矩必须联合起来，满足被动关节部分的方程 $M_{passive}\dot{v} + h_{passive} = J_{passive}^T F_c$。这就是为什么我们只能用解整机动力学的方式（Pinocchio + QP）来获得平滑的前馈扭矩。

## 核心技术栈
- **Pinocchio** (Python API):
  极速刚体动力学库。负责读取 `marsdog.urdf`，计算惯性矩阵 $M$、非线性项 $h$ (科里奥利与离心力、重力)、空间雅可比 $J$ 及雅可比微商 $\dot{J}$。由于是 C++ 底层，其 `computeAllTerms` 开销通常远小于 1ms，完美满足 200Hz+ 的控制频率。
- **qpsolvers (OSQP)**:
  用作核心 QP 优化器的前端。解出摩擦锥内部的接触力 $F_c$ 和电机扭矩 $\tau$。

## 数据流向与契约 (Data Flow)
新动力学模块完全服从目前的 `ControlOutput` 契约（定义在 `src/marsdog_control/core/types.py`）：
1. 运动学层 (`MotionPlanner` / `GaitController`) 依然生成前向开环的目标 $(q, \dot{q})$。
2. 动力学层 (`wbc.py` 中的 `_apply_wbc`) 接收：
   - 目标躯干加速度 $\ddot{p}_{base}$。
   - 足端着地状态 `leg_is_stance`。
   - `Pinocchio` 导出的模型 $M, h, J$。
3. `wbc.py` 内的 QP 求解器算出最优扭矩 $\tau_{opt}$。
4. `executor.py` 将 $\tau_{opt}$ 填充进 `trq_ff` 字典。
5. 最底层的执行器原封不动地将 `trq_ff` 作为 MIT 五元组之一，下发到 CAN 总线或 MuJoCo SimBackend。

## 模块边界 (WBC 路径)

```
VelocityCommand / UserCommand.vx,turn
        ↓
SoftTrotSchedule → amp, period, stance, turn, vel_cmd(SI)
        ↓
GaitController → MotionTarget(q,dq)
        ↓
ContactSchedule  →  measured/scheduled stance + force_scale
ForcePlanner     →  SRB-MPC + continuous horizon×f_max + edge scale + EMA/rate → f_c_des
WholeBodyController (QP) → τ (stance + swing foot accel track)
BaseStateEstimator → vx,vy,vz,z (default; truth 仅调试)
DynamicsTelemetry → ring buffers
```

接触软着陆（仿真/实机冲击）:
- MPC horizon 用连续 `_phase_edge_scale`∈[0,1] 缩放 `f_max`，避免 TD/LO 硬 0/1 翻转让 QP 瞬时重分配力
- `contact_edge_blend≈0.10`（约 10% 周期）、`force_lpf_alpha≈0.12`、`max_df_dt≈900 N/s`
- 运动学足端已是 minimum-jerk；观感跳变多来自力/任务边沿，而非轨迹几何

`CommandExecutor` 只做编排；接触日程、力规划、QP、遥测、速度→步态各自独立。
入口汇总见 `control/locomotion_stack.py`。

动力学接管程度（仿真）：
- 支撑力 / 摆动足加速度 / 基座 PD：WBC `trq_ff`
- 足端几何与相位：运动学 gait（`q`）
- 关节 MIT kp：WBC 下 `leg_kp_scale≈0.65`，让 τ_ff 更主导

实机契约：`RobotState.joint_vel` 由 Backend 填 URDF 空间速度；`vel_xyz` 由估计器填（不依赖仿真真值）。
WBC 优先读 `gait.vel_cmd`（由 SoftTrotSchedule 写入），不再只靠 amp/period 反推。
