from marsdog_control.motion.legacy_gait_controllers import TrotController
import time

trot = TrotController()
trot.target_amp_front = 0.0216
for i in range(10):
    trot.get_targets(i * 0.01)
    print(f"frame {i}: amp_front = {trot.amp_front:.5f}")
