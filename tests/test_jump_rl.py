from marsdog_control.motion.gait_controller import JumpController
jc = JumpController()
print(getattr(jc, "family", None))
