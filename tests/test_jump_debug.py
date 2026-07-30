import numpy as np
from marsdog_control.motion.gait_controller import JumpController

def test_jump_debug():
    jc = JumpController()
    jc.request_jump(True)
    
    # advance to PUSH
    jc._advance(0.0) # IDLE -> CROUCH
    jc._advance(jc.crouch_s) # CROUCH -> PUSH
    
    print("Phase:", jc.phase)
    
    H = 10
    dt_mpc = 0.03
    t_rel = jc.crouch_s + 0.05 # middle of PUSH
    
    contact_h = np.zeros(4 * H, dtype=float)
    for k in range(H):
        t_k = t_rel + k * dt_mpc
        jfs = float(jc.jump_force_scale_at(t_k))
        for li in range(4):
            contact_h[k * 4 + li] = jfs
            
    print("contact_h:", contact_h[::4])
    
    vz_cmd = jc.desired_vz(t_rel)
    print("vz_cmd:", vz_cmd)

if __name__ == "__main__":
    test_jump_debug()
