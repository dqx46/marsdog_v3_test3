import math
from typing import Dict, Tuple

from marsdog_control.motion.jacobian import jacobian_front_z, jacobian_rear_z

class VmcConfig:
    def __init__(self, 
                 kp_z: float = 3000.0,
                 kd_z: float = 60.0,
                 kp_roll: float = 30.0,
                 kd_roll: float = 2.0,
                 mass: float = 12.0,
                 track_width: float = 0.24,
                 h_err_clamp_m: float = 0.02,
                 min_support_ratio: float = 0.35):
        self.kp_z = kp_z
        self.kd_z = kd_z
        self.kp_roll = kp_roll
        self.kd_roll = kd_roll
        self.mass = mass
        self.track_width = track_width
        # FK(q_cur)-FK(q_tgt) 在软 KP 跟踪差时会偏大；钳位避免弹簧项吞掉重力前馈
        self.h_err_clamp_m = h_err_clamp_m
        # 支撑腿至少保留该比例的 mg/4，防止 Fz 被钳到 0
        self.min_support_ratio = min_support_ratio

class DecoupledVMC:
    def __init__(self, config: VmcConfig):
        self.config = config
        self.G = 9.81

    def compute_fz(self, 
                   leg_z_targets: Dict[str, float],
                   leg_z_current: Dict[str, float],
                   leg_vz_current: Dict[str, float],
                   roll: float,
                   roll_rate: float,
                   roll_target: float = 0.0,
                   leg_is_stance: Dict[str, bool] = None) -> Dict[str, float]:
        """
        计算每条腿需要向躯干提供的向上的力 Fz (N)。
        Z坐标系约定: hip 为 0, Z 轴向上 (因此 leg_z_current 均为负值, 如 -0.24)。
        """
        if leg_is_stance is None:
            leg_is_stance = {leg: True for leg in ['fl', 'fr', 'rl', 'rr']}
            
        # 计算当前有多少条腿在支撑 (至少为 2，避免除零或单腿承担过大前馈)
        stance_legs = sum(1 for v in leg_is_stance.values() if v)
        stance_legs = max(2, stance_legs)

        # 前馈: 动态支撑体重 (只分配给站立腿)
        ff_z = (self.config.mass * self.G) / stance_legs
        
        # Roll 轴 PD 控制
        # 目标是产生恢复力矩 T_roll
        # roll > 0 表示右倾, 需要产生负的恢复力矩 (逆时针)
        tau_roll = self.config.kp_roll * (roll_target - roll) - self.config.kd_roll * roll_rate
        
        # 差分分配到左右腿: tau_roll = Fz_left * (W/2) - Fz_right * (W/2)
        # 所以 dFz_left = tau_roll / W, dFz_right = -tau_roll / W
        delta_fz_roll = tau_roll / self.config.track_width
        
        fz_cmd = {}
        h_clamp = max(0.0, float(self.config.h_err_clamp_m))
        f_floor = ff_z * max(0.0, min(1.0, float(self.config.min_support_ratio)))
        for leg in ['fl', 'fr', 'rl', 'rr']:
            if not leg_is_stance.get(leg, True):
                # 摆动腿不参与 VMC 支撑，防止在空中被拉直
                fz_cmd[leg] = 0.0
                continue

            # 腿长误差代理体高: hip=0、Z 向上，足端 z 为负。
            # current=-0.20, target=-0.24 → 腿变短/躯干偏低 → H_err>0 → 向上推。
            H_err = leg_z_current[leg] - leg_z_targets[leg]
            if h_clamp > 0.0:
                H_err = max(-h_clamp, min(h_clamp, H_err))

            # 速度误差: 期望速度设为 0。V_err = 0 - V_current。
            V_err = -leg_vz_current.get(leg, 0.0)

            fz_spring = self.config.kp_z * H_err + self.config.kd_z * V_err

            if 'l' in leg:
                fz_leg = ff_z + fz_spring + delta_fz_roll
            else:
                fz_leg = ff_z + fz_spring - delta_fz_roll

            # 地面只能推不能拉；同时保留最小重力支撑，避免软腿跟踪误差把 Fz 打成 0
            fz_cmd[leg] = max(f_floor, fz_leg)

        return fz_cmd

    def compute_joint_torques(self, 
                              fz_cmd: Dict[str, float],
                              urdf_angles: Dict[str, Dict[str, float]]) -> Dict[str, Dict[str, float]]:
        """
        使用解析雅可比, 将期望的支撑力 Fz 映射为各关节的前馈力矩(URDF Nm)。
        """
        torques = {'fl': {}, 'fr': {}, 'rl': {}, 'rr': {}}
        
        for leg in ['fl', 'fr', 'rl', 'rr']:
            Fz = fz_cmd.get(leg, 0.0)
            angles = urdf_angles.get(leg, {})
            
            if leg in ['fl', 'fr']:
                hip = angles.get('hip_pitch', 0.0)
                calf = angles.get('calf', 0.0)
                tarsus = angles.get('tarsus', 0.0)
                Jz = jacobian_front_z(hip, calf, tarsus)
                
                # tau_motor = - J^T * F_ext_foot
                # 外界(地面)对足端施加的力就是 Fz (向上)。所以 F_ext_foot_z = Fz
                torques[leg]['hip_pitch'] = max(-12.0, min(12.0, -Jz[0] * Fz))
                torques[leg]['calf']      = max(-12.0, min(12.0, -Jz[1] * Fz))
                # 达妙主动 tarsus: 用自身的高 KP 去保持参考位置，暂不叠加 VMC 算出的低频支持力
                # torques[leg]['tarsus']    = -Jz[2] * Fz
                
            else:
                thigh = angles.get('thigh', 0.0)
                calf = angles.get('calf', 0.0)
                Jz = jacobian_rear_z(thigh, calf)
                
                torques[leg]['thigh'] = max(-12.0, min(12.0, -Jz[0] * Fz))
                torques[leg]['calf']  = max(-12.0, min(12.0, -Jz[1] * Fz))
                
        return torques
