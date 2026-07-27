import math
import json
from marsdog_control.motion.legacy_gait_controllers import TrotController
from marsdog_control.motion.kinematics import (
    FL_THIGH_LEN, FL_SHIN_LEN, WAIST_Z, FL_HIP_Z,
    RL_THIGH_LEN, RL_SHIN_LEN, RL_FOOT_LEN, RL_HIP_Z,
    ik_front_leg, ik_rear_leg
)

ctrl = TrotController(body_height=0.25, period=0.6, amp_front=0.22, amp_rear=0.28, step_height=0.03)

data = []

N = 60
for i in range(N):
    t = (i/N) * 0.6
    
    # FL
    theta_fl = ctrl._theta('fl', t)
    hip_u, h = ctrl._hip_and_height(theta_fl, ctrl.amp_front)
    calf_u = ik_front_leg(h, hip_u)
    
    fl_knee_x = -FL_THIGH_LEN * math.sin(hip_u)
    fl_knee_z = -FL_THIGH_LEN * math.cos(hip_u)
    
    fl_foot_x = fl_knee_x - FL_SHIN_LEN * math.sin(hip_u + calf_u)
    fl_foot_z = fl_knee_z - FL_SHIN_LEN * math.cos(hip_u + calf_u)
    
    # RL
    theta_rl = ctrl._theta('rl', t)
    thigh_u, h_r = ctrl._hip_and_height(theta_rl, ctrl.amp_rear)
    calf_u_rl = ik_rear_leg(h_r, thigh_u)
    
    rl_knee_x = -(RL_THIGH_LEN + RL_FOOT_LEN) * math.sin(thigh_u)
    rl_knee_z = -(RL_THIGH_LEN + RL_FOOT_LEN) * math.cos(thigh_u)
    
    rl_foot_x = rl_knee_x - RL_SHIN_LEN * math.sin(thigh_u + calf_u_rl)
    rl_foot_z = rl_knee_z - RL_SHIN_LEN * math.cos(thigh_u + calf_u_rl)

    data.append({
        't': t,
        'fl_hip': [0, 0],
        'fl_knee': [fl_knee_x, fl_knee_z],
        'fl_foot': [fl_foot_x, fl_foot_z],
        'rl_hip': [0, 0],
        'rl_knee': [rl_knee_x, rl_knee_z],
        'rl_foot': [rl_foot_x, rl_foot_z]
    })

with open('traj_data.json', 'w') as f:
    json.dump(data, f)
