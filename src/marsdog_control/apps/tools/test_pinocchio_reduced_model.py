import pinocchio as pin
import numpy as np
import os

def main():
    urdf_path = os.path.abspath(os.path.join(
        os.path.dirname(__file__), 
        "../../../../marsdog/urdf/marsdog.urdf"
    ))
    
    # 1. Load full model
    model = pin.buildModelFromUrdf(urdf_path, pin.JointModelFreeFlyer())
    data = model.createData()
    print("=== 全维度原始动力学模型 (Full Model) ===")
    print(f"全模型 q 维度 (nq): {model.nq}")
    print(f"全模型 v 维度 (nv): {model.nv}")
    
    # 计算初始状态的总质量和各个连杆的质量
    pin.computeTotalMass(model, data)
    print(f"整机总质量: {data.mass[0]:.4f} kg")

    # 2. Identify joints to lock
    # 我们需要锁定的关节包括：脖子、头部、尾巴的所有自由度
    joints_to_lock = [
        # Neck and Head
        "neck_pitch_joint",
        "head_roll_joint",
        "head_yaw_joint",
        "head_pitch_joint",
        # Tail
        "tail1_pitch_joint",
        "tail1_yaw_joint",
        "tail2_pitch_joint",
        "tail2_yaw_joint",
        "tail3_pitch_joint",
        "tail3_yaw_joint",
        "tail4_pitch_joint",
        "tail4_yaw_joint",
        "tail5_pitch_joint",
        "tail5_yaw_joint",
        "tail6_pitch_joint",
        "tail6_yaw_joint",
        "tail7_pitch_joint",
        "tail7_yaw_joint",
        "tail8_pitch_joint",
        "tail8_yaw_joint",
        "tail9_pitch_joint",
        "tail9_yaw_joint",
        "tail10_pitch_joint",
        "tail10_yaw_joint",
        "tail11_pitch_joint",
        "tail11_yaw_joint",
        "tail12_pitch_joint",
        "tail12_yaw_joint"
    ]
    
    # 找到这些关节的 ID
    list_of_joints_to_lock_ids = []
    for j_name in joints_to_lock:
        if model.existJointName(j_name):
            list_of_joints_to_lock_ids.append(model.getJointId(j_name))
        else:
            print(f"警告：未在 URDF 中找到关节 {j_name}")
            
    # 设置锁定时的关节参考位置 (q_ref)，对于头部和尾巴，直接锁定在零位
    q_ref = pin.neutral(model)
    
    # 3. Build the reduced model
    reduced_model = pin.buildReducedModel(model, list_of_joints_to_lock_ids, q_ref)
    reduced_data = reduced_model.createData()
    
    print("\n=== 降维四足动力学模型 (Reduced Model for NMPC) ===")
    print(f"降维后 q 维度 (nq): {reduced_model.nq}")
    print(f"降维后 v 维度 (nv): {reduced_model.nv}  <-- 极大地降低了非线性优化的维度！")
    
    # 重新计算总质量
    pin.computeTotalMass(reduced_model, reduced_data)
    print(f"降维模型总质量: {reduced_data.mass[0]:.4f} kg (动力学守恒：质量已完美传递到躯干)")
    
    # 打印保留下来的关节，验证是否只剩下浮动基和四条腿
    print("\n保留的核心动力学关节:")
    for j in reduced_model.names:
        print(f" - {j}")

if __name__ == "__main__":
    main()
