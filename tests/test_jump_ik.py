from marsdog_control.motion.gait_controller import JumpController
jc = JumpController()
jc.request_jump(True)
for i in range(10):
    t = i * 0.03
    targets = jc.get_targets(t)
    print(f"t={t:.3f} body_height={jc.body_height:.3f} h_cmd={jc._height_cmd:.3f} fl_hip={targets[1]:.3f} fl_thigh={targets[2]:.3f} fl_calf={targets[3]:.3f}")
