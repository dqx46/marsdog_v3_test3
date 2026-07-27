import pinocchio as pin
import numpy as np
import math
import os

from marsdog_control.motion import kinematics as K

def main():
    urdf_path = os.path.join(
        os.path.dirname(__file__), 
        "../../../../marsdog/urdf/marsdog.urdf"
    )
    
    # Load model with FreeFlyer (for floating base dynamics)
    model = pin.buildModelFromUrdf(urdf_path, pin.JointModelFreeFlyer())
    data = model.createData()
    
    print(f"Model: {model.name}")
    print(f"nq: {model.nq}, nv: {model.nv}")
    
    # We will test a standard standing posture
    # Default is zero for all joints
    q = pin.neutral(model)
    
    # Let's map some non-zero joint angles to see if FK matches
    # The joint configuration q in FreeFlyer:
    # q[0:3] = position XYZ
    # q[3:7] = quaternion (x,y,z,w)
    # q[7:] = joint angles
    
    # Let's define some test angles for front left (fl)
    fl_hip_pitch = 0.2
    fl_calf = -0.5
    fl_tarsus = 0.3
    
    # We need to find the indices of these joints in the q vector
    # pinocchio provides model.joints[id].idx_q
    def set_q(joint_name, value):
        if model.existJointName(joint_name):
            idx_q = model.joints[model.getJointId(joint_name)].idx_q
            q[idx_q] = value
            
    set_q('fl_hip_pitch_joint', fl_hip_pitch)
    set_q('fl_calf_joint', fl_calf)
    set_q('fl_tarsus_joint', fl_tarsus)
    
    # Compute FK
    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacements(model, data)
    
    # Get fl_foot_link position
    fl_foot_id = model.getFrameId("fl_foot_link")
    pos_pin_world = data.oMf[fl_foot_id].translation
    
    # In pinocchio with FreeFlyer, the base_link is at origin if q[0:7] is neutral.
    # The pure geometric FK in kinematics.py calculates relative to hip_pitch.
    # So we need the hip_pitch position in pinocchio.
    fl_hip_id = model.getFrameId("fl_hip_pitch_joint")
    pos_hip_pin = data.oMf[fl_hip_id].translation
    
    pos_rel_pin = pos_pin_world - pos_hip_pin
    
    print("--- Pinocchio FK ---")
    print(f"fl_foot_link (world): {pos_pin_world}")
    print(f"fl_hip_pitch_joint (world): {pos_hip_pin}")
    print(f"Relative (foot - hip): {pos_rel_pin}")
    
    # Calculate geometric FK from our kinematics.py
    # K.fk_front_3link returns (x, z) relative to hip_pitch, y is usually ignored or simple
    fk_x, fk_z = K.fk_front_3link(fl_hip_pitch, fl_calf, fl_tarsus)
    print("--- Geometric FK (kinematics.py) ---")
    print(f"fk_front_3link X: {fk_x}, Z: {fk_z}")
    
    # Compare
    print("--- Comparison ---")
    print(f"X diff: {abs(pos_rel_pin[0] - fk_x):.6f}")
    print(f"Z diff: {abs(pos_rel_pin[2] - fk_z):.6f}")

    # Compute Jacobian using Pinocchio
    pin.computeJointJacobians(model, data, q)
    J_foot = pin.getFrameJacobian(model, data, fl_foot_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
    print("--- Pinocchio Jacobian (World Aligned, 6xNV) ---")
    print("Shape:", J_foot.shape)
    
    # We can also compute M and h
    M = pin.crba(model, data, q)
    h = pin.nonLinearEffects(model, data, q, np.zeros(model.nv))
    print("--- Pinocchio Dynamics ---")
    print("M shape:", M.shape)
    print("h shape:", h.shape)
    print("Is M symmetric positive definite? (should be symmetric):", np.allclose(M, M.T))

if __name__ == "__main__":
    main()
