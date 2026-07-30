from marsdog_control.motion.gait_controller import JumpController, JumpPhase

def test_predict():
    jc = JumpController()
    # Let's write a predict function
    def predict_jfs(jc, t_future):
        # simulate _advance logic without mutating
        t = t_future
        phase = jc.phase
        t0 = jc._phase_t0
        
        # We need to advance phase if t > t0 + dur
        while True:
            dur = jc._dur(phase)
            if t < t0 + dur or dur <= 1e-9:
                break
            # advance to next phase
            t0 += dur
            if phase == JumpPhase.IDLE:
                if jc.trigger or jc.auto_rejump:
                    phase = JumpPhase.CROUCH
                else:
                    break
            elif phase == JumpPhase.CROUCH: phase = JumpPhase.PUSH
            elif phase == JumpPhase.PUSH: phase = JumpPhase.FLIGHT
            elif phase == JumpPhase.FLIGHT: phase = JumpPhase.LAND
            elif phase == JumpPhase.LAND: phase = JumpPhase.RECOVER
            elif phase == JumpPhase.RECOVER: 
                phase = JumpPhase.IDLE
                break
                
        # now we have the predicted phase and t0
        if phase == JumpPhase.FLIGHT:
            return 0.0
        if phase == JumpPhase.PUSH:
            return 1.0
        if phase == JumpPhase.LAND:
            u = max(0.0, min(1.0, (t - t0) / jc._dur(phase)))
            return 0.25 + 0.75 * jc._smooth(u)
        return 1.0

    jc.request_jump(True)
    jc._advance(0.0)
    jc._advance(jc.crouch_s)
    
    print("Current phase:", jc.phase)
    t_rel = jc.crouch_s + 0.05
    for k in range(10):
        t_k = t_rel + k * 0.03
        print(f"k={k} t_k={t_k:.3f} jfs={predict_jfs(jc, t_k):.2f}")

if __name__ == "__main__":
    test_predict()
