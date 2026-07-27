import math

def _clamp(val: float, min_val: float, max_val: float) -> float:
    return max(min_val, min(val, max_val))

def _deadzone(val: float, dz: float) -> float:
    if val > dz:
        return val - dz
    elif val < -dz:
        return val + dz
    return 0.0


def _smoothstep01(t: float) -> float:
    t = _clamp(t, 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)


def imu_phase_gain(phase: float,
                     stance_ratio: float = 0.60,
                     td_window: float = 0.12,
                     td_gain: float = 0.35,
                     mid_gain: float = 1.0,
                     swing_gain: float = 0.70) -> float:
    """步态相位 IMU 反馈增益 — 触地/离地窗口降增益, 支撑中期全力闭环。

    Trot 换腿时支撑多边形跳变 → 姿态冲击; 此时满增益 IMU 易与延迟叠加成极限环。
    支撑中期(脚已踩实)再全力修正; 摆动相略降(对侧支撑腿仍在干活)。
    实机与仿真同一扰动机理, 故同一套门控。
    """
    phase %= 1.0
    sr = max(0.05, min(0.95, stance_ratio))
    w = max(0.02, min(sr * 0.5, td_window))

    if phase < sr:
        if phase < w:
            s = _smoothstep01(phase / w)
            return td_gain + (mid_gain - td_gain) * s
        if phase > sr - w:
            s = _smoothstep01((sr - phase) / w)
            return td_gain + (mid_gain - td_gain) * s
        return mid_gain
    return swing_gain

class ImuAttitudeController:
    """IMU 姿态控制器 (PID + Leaky Integrator 形)

    采用标准 PID 结构, 积分项带有泄漏衰减, 微分项(阻尼)有限幅防抖动。
    直接计算 Z 轴修正量 (dz), 正面反馈到站立/步态的高度控制中。
    """
    def __init__(self,
                 kp_roll:  float = 0.05,
                 kp_pitch: float = 0.04,
                 ki_roll:  float = 0.000,
                 ki_pitch: float = 0.000,
                 kd_roll:  float = 0.002,
                 kd_pitch: float = 0.002,
                 decay_rate:     float = 0.98,
                 fast_decay:     float = 0.90,
                 max_correction: float = 0.020,
                 deadzone_deg:   float = 1.5,
                 fall_guard_deg: float = 22.0,
                 gyro_ema:       float = 0.0,
                 damp_soft_mm:   float = 3.0,
                 damp_hard_mm:   float = 3.0,
                 damp_gyro_lo:   float = 20.0,
                 damp_gyro_hi:   float = 80.0,
                 p_boost:        float = 1.0,
                 p_sched_lo_deg: float = 6.0,
                 p_sched_hi_deg: float = 14.0,
                 roll_trim_mm:   float = 0.0,
                 pitch_trim_mm:  float = 0.0,
                 auto_trim:          bool  = False,
                 auto_trim_rate:     float = 0.12,
                 auto_trim_limit_mm: float = 12.0,
                 ff_phases:          int   = 1,
                 predict_lead_s:     float = 0.0,
                 prediction_max_s:   float = 0.080,
                 gyro_max_age_s:     float = 0.030,
                 dynamic_prediction: bool = True,
                 nominal_dt_s:       float = 0.005):
        self.kp_roll  = kp_roll
        self.kp_pitch = kp_pitch
        self.ki_roll  = ki_roll
        self.ki_pitch = ki_pitch
        self.kd_roll  = kd_roll
        self.kd_pitch = kd_pitch
        self.decay_rate     = decay_rate
        self.fast_decay     = fast_decay
        nominal_dt_s = max(1e-4, float(nominal_dt_s))
        self._decay_tau_s = (-nominal_dt_s / math.log(decay_rate)
                             if 0.0 < decay_rate < 1.0 else 0.0)
        self._fast_decay_tau_s = (-nominal_dt_s / math.log(fast_decay)
                                  if 0.0 < fast_decay < 1.0 else 0.0)
        self.max_correction = max_correction
        self.deadzone_rad   = math.radians(deadzone_deg)
        self.fall_guard_rad = math.radians(fall_guard_deg)

        # [P2] D 项 gyro 的额外 EMA 滤波系数 (0=关; (0,1)=对旧值的权重, 越大越平滑)。
        # 只滤 D 用的角速度, 不动 P 用的角度, 避免给比例项引入控制滞后。
        self.gyro_ema = _clamp(gyro_ema, 0.0, 0.99)
        self._gyro_ema_tau_s = (-nominal_dt_s / math.log(self.gyro_ema)
                                if self.gyro_ema > 0.0 else 0.0)
        self._gr_f = 0.0
        self._gp_f = 0.0

        # [B] 非线性阻尼: 小角速度沿用软限幅(不抽腿), 大角速度(振铃)放大限幅强力压制。
        # gyro 单位 deg/s。lo 以下=软限幅; hi 以上=硬限幅; 之间线性过渡。
        # damp_hard<=damp_soft 时退化为原来的恒定软限幅(默认关)。
        self.damp_soft   = damp_soft_mm / 1000.0
        self.damp_hard   = max(damp_hard_mm / 1000.0, damp_soft_mm / 1000.0)
        self.damp_gyro_lo = damp_gyro_lo
        self.damp_gyro_hi = max(damp_gyro_hi, damp_gyro_lo + 1e-3)

        # [D] P 项增益调度: 小倾角温和(防震荡), 大倾角(发散)自动加大恢复力救场。
        # p_boost<=1 时关闭(默认)。lo 以下增益=1, hi 以上=p_boost, 之间线性。
        self.p_boost       = max(1.0, p_boost)
        self.p_sched_lo    = p_sched_lo_deg
        self.p_sched_hi    = max(p_sched_hi_deg, p_sched_lo_deg + 1e-3)

        # [T] 静态配平: 绕过死区/泄漏的常量偏置, 抵消步态引起的直流姿态偏置。
        #   与 P/I/D 分工: P/D 压死区外快速抖动, I 记短期趋势, trim 扛持续 DC 歪。
        self.roll_trim  = roll_trim_mm / 1000.0   # roll 配平 (m), +=等效机身更"左高"
        self.pitch_trim = pitch_trim_mm / 1000.0  # pitch 配平 (m)

        # [AT] 在线自学习配平: 慢积分器学出步态直流 roll 偏置, 每台狗自适应(量产友好)。
        #   只学 roll(应为0); 用原始 roll 绕过死区; 慢速率只跟 DC 不追摆动; 上下限防跑飞。
        self.auto_trim       = auto_trim
        self.auto_trim_rate  = auto_trim_rate           # m/(rad·s)
        self.auto_trim_limit = auto_trim_limit_mm / 1000.0

        # [ILC] 相位前馈表: 学出 roll 随 gait 相位的确定性摆动并逐相位前馈抵消。
        #   ff_phases=1 → 退化为单值(=原 auto-trim, 只学直流); >1 → 按相位学整条曲线。
        #   前馈=零滞后零噪声, 专治"每步换对角就栽一下"的相位锁定摇摆, 不抖。
        self.ff_phases = max(1, int(ff_phases))
        self._roll_ff  = [0.0] * self.ff_phases  # 每相位学习到的前馈 (m); roll_trim 为手动静态偏置

        # [PRED] 延迟补偿: 用陀螺仪把姿态外推 predict_lead_s 秒喂给 P 项(超前校正),
        #   对冲 ~100ms 物理反馈延迟, 让 P 能更强而不因滞后振荡。0=关(原行为)。
        # predict_lead_s 现在表示执行器额外提前量；总外推时间还包含 angle 数据年龄。
        self.predict_lead_s = max(0.0, predict_lead_s)
        self.prediction_max_s = max(0.0, prediction_max_s)
        self.gyro_max_age_s = max(0.0, gyro_max_age_s)
        self.dynamic_prediction = bool(dynamic_prediction)
        self._prediction_lead_s = 0.0

        self._roll_i  = 0.0   # roll 积分项 (m)
        self._pitch_i = 0.0   # pitch 积分项 (m)
        self._prev_roll_c = None   # 上一周期 roll 校正 (m), 用于斜率限制
        self._prev_pitch_c = None  # 上一周期 pitch 校正 (m), 用于斜率限制
        
        self._roll_out  = 0.0   # roll 总输出 (m) (仅用于 logging)
        self._pitch_out = 0.0   # pitch 总输出 (m) (仅用于 logging)
        
        self._enabled = False
        self._last_phase_gain = 1.0
        self._components = {
            "p_roll": 0.0, "i_roll": 0.0, "d_roll": 0.0, "trim_roll": 0.0,
            "p_pitch": 0.0, "i_pitch": 0.0, "d_pitch": 0.0, "trim_pitch": 0.0,
        }

    @property
    def enabled(self):
        return self._enabled

    def enable(self):
        self._enabled = True
        self._roll_i = 0.0
        self._pitch_i = 0.0
        self._prev_roll_c = None
        self._prev_pitch_c = None

    def disable(self):
        self._enabled = False
        self.reset()

    def reset(self):
        """重置积分状态 (模式切换时调用, 防止残留补偿干扰新步态)。"""
        self._roll_i = 0.0
        self._pitch_i = 0.0
        self._prev_roll_c = None
        self._prev_pitch_c = None

    def _p_gain_scale(self, angle_rad: float) -> float:
        """[D] P 项增益调度: |angle|<=lo → 1.0; >=hi → p_boost; 之间线性。
        小倾角温和防震荡, 大倾角加大恢复力救发散。"""
        if self.p_boost <= 1.0:
            return 1.0
        a = abs(math.degrees(angle_rad))
        if a <= self.p_sched_lo:
            return 1.0
        if a >= self.p_sched_hi:
            return self.p_boost
        f = (a - self.p_sched_lo) / (self.p_sched_hi - self.p_sched_lo)
        return 1.0 + (self.p_boost - 1.0) * f

    def _damp_d(self, kd_base: float, gyro_rps: float) -> float:
        """[B] 非线性阻尼 D 项 (m)。传入 gyro 单位 rad/s, 阈值 lo/hi 以 deg/s 设定。

        小角速度(|gyro|<=lo): 沿用软限幅 + 原 kd, 不抽腿。
        大角速度(振铃, |gyro|>=hi): 等效 kd 与限幅同时放大, 强力压制。
        中间线性过渡。damp_hard<=damp_soft 时退化为原恒定软限幅(默认关)。
        单位修正: 之前把 rad/s 当 deg/s + kd 太小打不到限幅, 导致 B 完全失效。
        """
        if self.damp_hard <= self.damp_soft:
            return _clamp(kd_base * gyro_rps, -self.damp_soft, self.damp_soft)
        a_dps = abs(math.degrees(gyro_rps))
        if a_dps <= self.damp_gyro_lo:
            return _clamp(kd_base * gyro_rps, -self.damp_soft, self.damp_soft)
        f = min(1.0, (a_dps - self.damp_gyro_lo) /
                (self.damp_gyro_hi - self.damp_gyro_lo))
        lim = self.damp_soft + (self.damp_hard - self.damp_soft) * f
        # 配套放大等效 kd: 使在 hi 处 kd*gyro 恰好能到达 hard 限幅
        hi_rps = math.radians(self.damp_gyro_hi)
        kd_hi = self.damp_hard / hi_rps if hi_rps > 1e-6 else kd_base
        kd_eff = kd_base + (max(kd_base, kd_hi) - kd_base) * f
        return _clamp(kd_eff * gyro_rps, -lim, lim)

    def set_state(self, roll_out: float, pitch_out: float):
        """状态继承 (stand<->trot 切换时保持补偿连续, 参考项目同款)。"""
        self._roll_i = _clamp(roll_out, -self.max_correction, self.max_correction)
        self._pitch_i = _clamp(pitch_out, -self.max_correction, self.max_correction)

    def update(self, roll: float, pitch: float,
               gyro_roll: float, gyro_pitch: float,
               freeze_integrator: bool = False,
               slew_limit_m_per_s: float = 0.0,
               dt_s: float = 0.005,
               gait_phase: float = 0.0,
               trim_gain: float = 1.0,
               phase_gain: float = 1.0,
               angle_age_s: float = 0.0,
               gyro_age_s: float = 0.0) -> dict:
        """PID 更新, 返回各腿 Z 修正。

        Args:
            roll, pitch: 机身姿态角 (rad), 已扣除有意倾斜(由 walk.py 处理)。
            gyro_roll, gyro_pitch: 机身角速度 (rad/s)。
        Returns:
            {'fl','fr','rl','rr': dz(m)}, dz>0=缩短, dz<0=伸长。
        """
        if not self._enabled:
            return {'fl': 0.0, 'fr': 0.0, 'rl': 0.0, 'rr': 0.0}

        dt_s = max(1e-4, min(0.1, dt_s))

        # [PRED] 总外推时间 = 角度数据年龄 + 执行器额外提前量，并受硬上限约束。
        # gyro 过旧时禁止外推，避免用失效角速度放大姿态误差。
        lead = self.predict_lead_s
        if self.dynamic_prediction:
            lead += max(0.0, angle_age_s)
        lead = _clamp(lead, 0.0, self.prediction_max_s)
        if gyro_age_s > self.gyro_max_age_s:
            lead = 0.0
        self._prediction_lead_s = lead
        roll_pred = roll + gyro_roll * lead
        pitch_pred = pitch + gyro_pitch * lead

        # 必须使用平滑死区(_deadzone), 否则在死区边缘会发生阶跃跳变导致高频震荡
        roll_eff = _deadzone(roll_pred, self.deadzone_rad)     # P 用预测(超前)
        pitch_eff = _deadzone(pitch_pred, self.deadzone_rad)
        roll_eff_i = _deadzone(roll, self.deadzone_rad)        # I/衰减 用原始(慢, 不引噪)
        pitch_eff_i = _deadzone(pitch, self.deadzone_rad)

        # --- 比例项 (P) 带增益调度 ---
        # 用预测角度判断倾角大小, 大倾角自动加大恢复力救发散
        p_roll = self.kp_roll * roll_eff * self._p_gain_scale(roll_pred)
        p_pitch = self.kp_pitch * pitch_eff * self._p_gain_scale(pitch_pred)

        # --- 积分项 (I) 带有泄漏 ---
        # [E] 触地窗口可选冻结积分: 避免落地冲击瞬时误差把积分项“打歪”后残留。
        if not freeze_integrator:
            self._roll_i += self.ki_roll * roll_eff_i * dt_s
            self._pitch_i += self.ki_pitch * pitch_eff_i * dt_s

        # 泄漏衰减: 死区外缓慢衰减(留记忆), 死区内快速归零(防长尾)
        tau_r = self._decay_tau_s if roll_eff_i != 0.0 else self._fast_decay_tau_s
        tau_p = self._decay_tau_s if pitch_eff_i != 0.0 else self._fast_decay_tau_s
        self._roll_i *= math.exp(-dt_s / tau_r) if tau_r > 0.0 else 0.0
        self._pitch_i *= math.exp(-dt_s / tau_p) if tau_p > 0.0 else 0.0

        # --- 微分项/阻尼 (D) ---
        # [P2] 可选对 gyro 做 EMA 去抖 (只影响 D, 不影响 P 的响应速度)
        if self.gyro_ema > 0.0:
            a = math.exp(-dt_s / self._gyro_ema_tau_s)
            self._gr_f = a * self._gr_f + (1.0 - a) * gyro_roll
            self._gp_f = a * self._gp_f + (1.0 - a) * gyro_pitch
            gr_d, gp_d = self._gr_f, self._gp_f
        else:
            gr_d, gp_d = gyro_roll, gyro_pitch
        # [B] 非线性阻尼: 小角速度软(不抽腿), 大角速度(振铃)放大等效kd+限幅强力压制
        d_roll = self._damp_d(self.kd_roll, gr_d)
        d_pitch = self._damp_d(self.kd_pitch, gp_d)
        
        # 平滑过渡阻尼：在死区边缘附近平滑放大，防止跨越死区时 d_roll 瞬间突变引发震荡
        fade_roll = min(1.0, abs(roll_eff) / math.radians(1.0))
        fade_pitch = min(1.0, abs(pitch_eff) / math.radians(1.0))
        
        d_roll *= fade_roll
        d_pitch *= fade_pitch

        mc = self.max_correction
        self._roll_i = _clamp(self._roll_i, -mc, mc)
        self._pitch_i = _clamp(self._pitch_i, -mc, mc)

        # [AT/ILC] 在线自学习前馈: 按 gait 相位分桶, 慢积分原始 roll, 前馈抵消。
        #   ff_phases=1 → 单桶=学直流(原 auto-trim); >1 → 逐相位学摇摆曲线(ILC)。
        #   必须与 P 项(kp*roll, 已验证修正正确)同号才是负反馈:
        #   roll<0(右歪) → ff 变负 → out_roll 变负 → 拉 roll 回 0; 平衡点 roll≈0 自停。
        #   (取反号会变正反馈自激发散 → 曾导致腿疯狂抽搐, 切勿改回 -roll)
        #   前馈=零滞后零噪声, 不注入抖动。落地窗口冻结防冲击污染学习值。
        n = self.ff_phases
        ff_bin = int(gait_phase * n) % n if n > 1 else 0
        if self.auto_trim and not freeze_integrator:
            # 单桶被访问频率 1/n, 乘 n 使墙钟收敛速度与相位数无关
            rate = self.auto_trim_rate * (n if n > 1 else 1)
            delta = rate * roll * dt_s
            lim = self.auto_trim_limit
            # [Q滤波] 把每次调整抹到相邻桶(三角核), 禁止相邻桶正负打架撞轨(防ILC因
            #   物理延迟发散)。n=1 时三个偏移都落到 0 桶, 退化为原单值行为。
            for off, w in ((-1, 0.25), (0, 0.5), (1, 0.25)):
                j = (ff_bin + off) % n
                self._roll_ff[j] = _clamp(self._roll_ff[j] + w * delta, -lim, lim)

        # [T/ILC] 前馈直接加入输出 (不受死区影响): 手动静态 roll_trim + 学习到的相位前馈
        #   trim_gain: 配平项随步态振幅斜坡渐入 (0→1)。稳态直流偏置在起步时尚未发育,
        #   若满幅怼上会过度修正把狗弹起("起飞"); 按振幅比例施加则永不过修。
        #   注意: 只门控配平/前馈, 不动 P/I/D 反馈, 反馈仍可即时压扰动。
        trim_r = trim_gain * (self.roll_trim + self._roll_ff[ff_bin])
        trim_p = trim_gain * self.pitch_trim
        pg = _clamp(phase_gain, 0.0, 1.0)
        # 相位门控只缩放 P/I/D 反馈; 配平/ILC 前馈(trim)保持 — 学出的是 DC/周期偏置
        out_roll = pg * (p_roll + self._roll_i + d_roll) + trim_r
        out_pitch = pg * (p_pitch + self._pitch_i + d_pitch) + trim_p

        self._last_phase_gain = pg
        self._components = {
            "p_roll": pg * p_roll, "i_roll": pg * self._roll_i,
            "d_roll": pg * d_roll, "trim_roll": trim_r,
            "p_pitch": pg * p_pitch, "i_pitch": pg * self._pitch_i,
            "d_pitch": pg * d_pitch, "trim_pitch": trim_p,
        }
        self._roll_out = out_roll
        self._pitch_out = out_pitch

        # ── 摔倒保护: 倾角过大时补偿平滑归零 ───────────────────────────
        tilt = max(abs(roll), abs(pitch))
        if tilt > self.fall_guard_rad:
            guard = max(0.0, 1.0 - (tilt - self.fall_guard_rad) / self.fall_guard_rad)
        else:
            guard = 1.0

        roll_c = out_roll * guard
        pitch_c = out_pitch * guard

        # [E] 校正斜率限制: 限制每周期校正变化率, 抑制落地瞬间“猛拉”导致的机械冲击。
        if slew_limit_m_per_s > 1e-9:
            dt = max(1e-4, dt_s)
            max_step = slew_limit_m_per_s * dt
            if self._prev_roll_c is None:
                self._prev_roll_c = roll_c
            if self._prev_pitch_c is None:
                self._prev_pitch_c = pitch_c
            dr = roll_c - self._prev_roll_c
            dp = pitch_c - self._prev_pitch_c
            dr = _clamp(dr, -max_step, max_step)
            dp = _clamp(dp, -max_step, max_step)
            roll_c = self._prev_roll_c + dr
            pitch_c = self._prev_pitch_c + dp
            self._prev_roll_c = roll_c
            self._prev_pitch_c = pitch_c
        else:
            self._prev_roll_c = roll_c
            self._prev_pitch_c = pitch_c

        # IMU: roll>0 = 左HIGH → 缩短左腿(dz>0), 伸长右腿(dz<0)
        # IMU: pitch>0 = 抬头   → 缩短前腿(dz>0), 伸长后腿(dz<0)
        dz_fl = _clamp(+roll_c + pitch_c, -mc, mc)
        dz_fr = _clamp(-roll_c + pitch_c, -mc, mc)
        dz_rl = _clamp(+roll_c - pitch_c, -mc, mc)
        dz_rr = _clamp(-roll_c - pitch_c, -mc, mc)

        return {'fl': dz_fl, 'fr': dz_fr, 'rl': dz_rl, 'rr': dz_rr}

    def get_roll_ff_mm(self):
        """返回学习到的相位前馈表 (mm), 用于持久化。"""
        return [round(x * 1000.0, 3) for x in self._roll_ff]

    def set_roll_ff_mm(self, table):
        """加载相位前馈表 (mm)。长度不匹配则忽略(相位数变了需重学)。"""
        if not table or len(table) != self.ff_phases:
            return False
        lim = self.auto_trim_limit
        self._roll_ff = [max(-lim, min(lim, float(v) / 1000.0)) for v in table]
        return True

    @property
    def roll_ff_span_mm(self):
        """前馈表峰峰值 (mm), 反映相位摇摆被抵消的幅度。"""
        return (max(self._roll_ff) - min(self._roll_ff)) * 1000.0

    @property
    def roll_out(self):
        return self._roll_out

    @property
    def pitch_out(self):
        return self._pitch_out

    @property
    def prediction_lead_s(self):
        return self._prediction_lead_s

    @property
    def components(self):
        return dict(self._components)

    def describe(self) -> str:
        damp = (f"damp={self.damp_soft*1000:.0f}→{self.damp_hard*1000:.0f}mm"
                f"@{self.damp_gyro_lo:.0f}-{self.damp_gyro_hi:.0f}dps"
                if self.damp_hard > self.damp_soft
                else f"damp={self.damp_soft*1000:.0f}mm")
        boost = (f"  Pboost={self.p_boost:.1f}@{self.p_sched_lo:.0f}-{self.p_sched_hi:.0f}°"
                 if self.p_boost > 1.0 else "")
        trim = (f"  trim(r/p)={self.roll_trim*1000:+.1f}/{self.pitch_trim*1000:+.1f}mm"
                if (self.roll_trim or self.pitch_trim) else "")
        if self.auto_trim:
            mode = f"ILC×{self.ff_phases}相位" if self.ff_phases > 1 else "AUTOtrim"
            trim += (f"  {mode}(rate={self.auto_trim_rate:.2f},"
                     f"±{self.auto_trim_limit*1000:.0f}mm)")
        pred = (f"  执行提前={self.predict_lead_s*1000:.0f}ms"
                f"+数据年龄(上限{self.prediction_max_s*1000:.0f}ms)"
                if self.predict_lead_s > 1e-6 else "")
        return (f"IMU_PID  P(r/p)={self.kp_roll:.3f}/{self.kp_pitch:.3f}  "
                f"I={self.ki_roll:.4f}  D={self.kd_roll:.4f}  "
                f"decay={self.decay_rate:.3f}  "
                f"max={self.max_correction*1000:.0f}mm  "
                f"dz={math.degrees(self.deadzone_rad):.1f}°  "
                f"guard={math.degrees(self.fall_guard_rad):.0f}°  "
                f"{damp}{boost}{trim}{pred}  "
                f"{'ON' if self._enabled else 'OFF'}")
