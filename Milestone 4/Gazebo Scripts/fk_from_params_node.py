#!/usr/bin/env python3
import rospy
import numpy as np
import math

# ============================================================
# 1) SYMBOLIC DH (for documentation / report / offline testing)
#    This is NOT used at runtime in the ROS loop.
# ============================================================

# (same as before – kept commented for report)
# import sympy as sp
# q1_sym, q2_sym, q3_sym, q4_sym = sp.symbols('q1 q2 q3 q4')
# L1_mm = 38.55
# L2_mm = 140
# L3_mm = 103
# L4_mm = 29.85
# ...  (keep your old symbolic block if you want it in the file)


# ============================================================
# 2) NUMERIC FORWARD KINEMATICS (URDF-BASED, NOW UP TO EE_frame)
# ============================================================

def forward_kinematics_func(q: np.ndarray) -> np.ndarray:
    """
    FK IMPLEMENTATION (URDF-BASED) → EE_frame

    Input:
        q : np.array(4,) in RADIANS
            q[0] -> Joint_1 angle   (q1)
            q[1] -> Joint_2 angle   (q2)
            q[2] -> Joint_3 angle   (q3)
            q[3] -> Joint_5 angle   (q4 = wrist pitch)

    Output:
        X : np.array(3,) -> [x, y, z] in meters
        expressed in the WORLD frame (same as /gazebo/link_states),
        for the link **EE_frame** (so it matches ee_listener.py).
    """

    q = np.array(q, dtype=float).flatten()
    q1, q2, q3, q4 = q   # q4 here is Joint_5 angle

    # ---------- helpers ----------
    def RotX(a):
        ca, sa = np.cos(a), np.sin(a)
        return np.array([
            [1, 0, 0],
            [0, ca, -sa],
            [0, sa,  ca]
        ])

    def RotY(a):
        ca, sa = np.cos(a), np.sin(a)
        return np.array([
            [ ca, 0, sa],
            [  0, 1,  0],
            [-sa, 0, ca]
        ])

    def RotZ(a):
        ca, sa = np.cos(a), np.sin(a)
        return np.array([
            [ca, -sa, 0],
            [sa,  ca, 0],
            [ 0,   0, 1]
        ])

    def T(R, p):
        Tmat = np.eye(4)
        Tmat[0:3, 0:3] = R
        Tmat[0:3, 3] = p
        return Tmat

    # ============================================================
    #  URDF-BASED CHAIN:
    #  World → Rotating_Waste → Arm_1 → Arm_2 → Arm_3 → Gripper → EE_frame
    # ============================================================

    # 0) World → Joint_1 (Rotating_Waste)
    #   We approximate the base height as 0.05 m (from link_states when all q=0).
    #   If you want it exact, read the <origin> of Joint_1 from the URDF.
    T0_1 = T(RotZ(q1), np.array([0.0, 0.0, 0.050]))

    # 1) Joint_1 → Joint_2 (Rotating_Waste → Arm_1)
    #   <joint name="Joint_2" type="revolute">
    #     <origin xyz="-2.7567E-05 0.0061999 0.04975" rpy="1.5708 0 0"/>
    #     <axis   xyz="0 0 1"/>
    origin_J2 = np.array([-2.7567e-05, 0.0061999, 0.04975])
    R_J2_fixed = RotZ(0.0) @ RotY(0.0) @ RotX(1.5708)
    R_J2 = R_J2_fixed @ RotZ(q2)        # axis="0 0 1"
    T1_2 = T(R_J2, origin_J2)

    # 2) Joint_2 → Joint_3 (Arm_1 → Arm_2)
    #   <joint name="Joint_3" type="revolute">
    #     <origin xyz="0 0.140 0.000" rpy="0 0 1.5708"/>
    #     <axis   xyz="0 0 -1"/>
    origin_J3 = np.array([0.0, 0.140, 0.0])
    R_J3_fixed = RotZ(1.5708)
    R_J3 = R_J3_fixed @ RotZ(-q3)       # axis="0 0 -1"
    T2_3 = T(R_J3, origin_J3)

    # 3) Joint_3 → Joint_4 (Arm_2 → Arm_3)
    #   <joint name="Joint_4" type="revolute">
    #     <origin xyz="0 0 0" rpy="-1.5708 0 -1.5708"/>
    #     <axis   xyz="0 0 -1"/>
    #   In the project Joint_4 is always 0 deg, so it’s just a fixed rotation.
    R_J4_fixed = RotZ(-1.5708) @ RotY(0.0) @ RotX(-1.5708)
    T3_4 = T(R_J4_fixed, np.array([0.0, 0.0, 0.0]))

    # 4) Joint_4 → Joint_5 (Arm_3 → Gripper)
    #   <joint name="Joint_5" type="revolute">
    #     <origin xyz="-0.001651 -0.0054574 0.13289" rpy="1.5708 0 0"/>
    #     <axis   xyz="0 0 1"/>
    origin_J5 = np.array([-0.001651, -0.0054574, 0.13289])
    R_J5_fixed = RotZ(0.0) @ RotY(0.0) @ RotX(1.5708)
    R_J5 = R_J5_fixed @ RotZ(q4)        # axis="0 0 1"
    T4_5 = T(R_J5, origin_J5)

    # 5) Gripper → EE_frame  (EE_frame_lock fixed joint)
    #   <joint name="EE_frame_lock" type="fixed">
    #       <origin xyz="x_EE y_EE z_EE" rpy="r_EE p_EE y_EE"/>
    #   </joint>
    #
    #  TODO: OPEN YOUR URDF AND COPY THESE VALUES EXACTLY
    #   - Go to the joint "EE_frame_lock" (or similar name).
    #   - Take its xyz (in meters) and rpy (in radians).
    #   - Put them below.
    #
    # Example structure (REPLACE the numbers with the real ones!):
    x_EE = 0.0      # <-- replace with EE_frame_lock origin x
    y_EE = 0.0      # <-- replace with EE_frame_lock origin y
    z_EE = 0.10     # <-- replace with EE_frame_lock origin z
    roll_EE  = 0.0  # <-- replace with rpy roll
    pitch_EE = 0.0  # <-- replace with rpy pitch
    yaw_EE   = 0.0  # <-- replace with rpy yaw

    origin_EE = np.array([x_EE, y_EE, z_EE])
    R_EE_fixed = RotZ(yaw_EE) @ RotY(pitch_EE) @ RotX(roll_EE)
    T5_EE = T(R_EE_fixed, origin_EE)

    # Final transform: World → EE_frame
    T0_EE = T0_1 @ T1_2 @ T2_3 @ T3_4 @ T4_5 @ T5_EE

    pos = T0_EE[0:3, 3]
    return pos


# ============================================================
# 3) ROS NODE: read rosparams, call FK, print X,Y,Z
# ============================================================

def fk_node():
    rospy.init_node("fk_from_params_node", anonymous=True)
    rate = rospy.Rate(2)  # 2 Hz

    rospy.loginfo("fk_from_params_node started. Reading Joint_1, Joint_2, Joint_3, Joint_5 (deg).")

    # Defaults in DEGREES (same style as joint_publisher_param.py)
    defaults_deg = {
        "Joint_1": 30.0,
        "Joint_2": 30.0,
        "Joint_3": -30.0,
        "Joint_4": 0.0,   # ignored in FK
        "Joint_5": 30.0,
    }

    while not rospy.is_shutdown():
        deg1 = float(rospy.get_param("Joint_1", defaults_deg["Joint_1"]))
        deg2 = float(rospy.get_param("Joint_2", defaults_deg["Joint_2"]))
        deg3 = float(rospy.get_param("Joint_3", defaults_deg["Joint_3"]))
        deg5 = float(rospy.get_param("Joint_5", defaults_deg["Joint_5"]))

        q_deg = np.array([deg1, deg2, deg3, deg5], dtype=float)
        q_rad = np.deg2rad(q_deg)

        X = forward_kinematics_func(q_rad)
        x, y, z = X

        rospy.loginfo_throttle(
            0.5,
            f"[Python FK] For joints (deg) "
            f"[J1={deg1:.1f}, J2={deg2:.1f}, J3={deg3:.1f}, J5={deg5:.1f}] "
            f"-> X = [{x:.4f}, {y:.4f}, {z:.4f}] m"
        )

        rate.sleep()


if __name__ == "__main__":
    fk_node()
