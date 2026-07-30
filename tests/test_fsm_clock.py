from marsdog_control.runtime.walk_controllers import build_runtime_fsm
import time
class DummyClock:
    def time(self): return time.time()

fsm = build_runtime_fsm(
    gait_controllers={},
    stand_controller=None,
    clock=DummyClock()
)
print(hasattr(fsm, "clock"))
