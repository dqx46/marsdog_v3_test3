import math
from typing import Tuple

from marsdog_control.motion.kinematics import (
    _FL_L1, _FL_L2S, _FL_L3,
    _FL_PHI1, _FL_PHI2S, _FL_PHI3,
    _RL_L1, _RL_L2,
    _RL_PHI1, _RL_PHI2, _RL_THETA0
)

def jacobian_front_z(hip: float, calf: float, tarsus: float = 0.0) -> Tuple[float, float, float]:
    """前腿 3-link Z轴解析雅可比: dz/d(hip, calf, tarsus)
    
    返回 (dz_dhip, dz_dcalf, dz_dtarsus)。
    """
    a1 = _FL_PHI1 - hip
    a2 = _FL_PHI2S - hip - calf
    a3 = _FL_PHI3 - hip - calf - tarsus
    
    cos_a1 = math.cos(a1)
    cos_a2 = math.cos(a2)
    cos_a3 = math.cos(a3)
    
    dz_dhip = -_FL_L1 * cos_a1 - _FL_L2S * cos_a2 - _FL_L3 * cos_a3
    dz_dcalf = -_FL_L2S * cos_a2 - _FL_L3 * cos_a3
    dz_dtarsus = -_FL_L3 * cos_a3
    
    return dz_dhip, dz_dcalf, dz_dtarsus

def jacobian_rear_z(thigh: float, calf: float) -> Tuple[float, float]:
    """后腿 2-link Z轴解析雅可比: dz/d(thigh, calf)
    
    返回 (dz_dthigh, dz_dcalf)。
    """
    t_eff = _RL_THETA0 + thigh
    a1 = _RL_PHI1 - t_eff
    a2 = _RL_PHI2 - t_eff - calf
    
    cos_a1 = math.cos(a1)
    cos_a2 = math.cos(a2)
    
    dz_dthigh = -_RL_L1 * cos_a1 - _RL_L2 * cos_a2
    dz_dcalf = -_RL_L2 * cos_a2
    
    return dz_dthigh, dz_dcalf
