from marsdog_control.motion.gait_controller import JumpController
jc = JumpController()
jc.request_jump(True)
for i in range(71):
    t = i * 0.005
    targets = jc.get_targets(t)
print(f"t={t:.3f} phase={jc.phase} h_cmd={jc._height_cmd:.3f} body_height={jc.body_height:.3f}")
