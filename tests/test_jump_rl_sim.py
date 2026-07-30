from marsdog_control.motion.gait_controller import JumpController
from marsdog_control.motion.motion_planner import build_motion_target
from marsdog_control.core.types import RobotState

class DummyFSM:
    def __init__(self):
        self.active_gait = JumpController()
        self.active_gait.request_jump(True)
        self.t_gait = 0.0
        self.blend_active = False
        self.mode = "jump"

fsm = DummyFSM()
state = RobotState()
smooth_tgt = {}
cur_pos = {}
online = list(range(1, 25))

for i in range(20):
    t_rel = i * 0.005
    fsm.active_gait._advance(t_rel) # simulate what walk_loop does? No, get_targets does it.
    # Wait, get_targets is called inside build_motion_target!
    # But wait, fsm.clock doesn't exist, so it uses time.time()!
    # Ah! In motion_planner.py:
    # if hasattr(fsm, "clock"): t_rel = fsm.clock.time() - fsm.t_gait
    # else: t_rel = time.time() - fsm.t_gait
    # We must provide clock to fsm or mock time.time!
    pass
