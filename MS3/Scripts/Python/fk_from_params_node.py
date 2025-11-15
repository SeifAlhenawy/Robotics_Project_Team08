#!/usr/bin/env python3
import rospy
import numpy as np
import math

# ============================================================
# 1) SYMBOLIC DH (for documentation / report / offline testing)
#    This is NOT used at runtime in the ROS loop.
# ============================================================

# If Sympy is installed, you can uncomment these lines and run this file
# as a normal Python script (NOT as a ROS node) to verify the DH-based T04.
#
# import sympy as sp
#
# # Define symbolic joint variables
# q1_sym, q2_sym, q3_sym, q4_sym = sp.symbols('q1 q2 q3 q4')
#
# # Define link lengths (in mm)
# L1_mm = 38.55   # Link 1
# L2_mm = 140     # Link 2
# L3_mm = 103     # Link 3
# L4_mm = 29.85   # End-effector link
#
# pi = sp.pi
#
# # Base to Joint 1
# T01 = sp.Matrix([
#     [sp.cos(q1_sym), -sp.sin(q1_sym)*sp.cos(pi/2), sp.sin(q1_sym)*sp.sin(pi/2), 0],
#     [sp.sin(q1_sym),  sp.cos(q1_sym)*sp.cos(pi/2), -sp.cos(q1_sym)*sp.sin(pi/2), 0],
#     [0,               sp.sin(pi/2),                sp.cos(pi/2),                L1_mm],
#     [0,               0,                           0,                           1]
# ])
#
# # Joint 1 to Joint 2
# T12 = sp.Matrix([
#     [sp.cos(pi/2 + q2_sym), -sp.sin(pi/2 + q2_sym)*sp.cos(pi),  sp.sin(pi/2 + q2_sym)*sp.sin(pi), L2_mm*sp.cos(pi/2 + q2_sym)],
#     [sp.sin(pi/2 + q2_sym),  sp.cos(pi/2 + q2_sym)*sp.cos(pi), -sp.cos(pi/2 + q2_sym)*sp.sin(pi), L2_mm*sp.sin(pi/2 + q2_sym)],
#     [0,                      sp.sin(pi),                       sp.cos(pi),                       0],
#     [0,                      0,                               0,                                1]
# ])
#
# # Joint 2 to Joint 3
# T23 = sp.Matrix([
#     [sp.cos(q3_sym), -sp.sin(q3_sym)*sp.cos(pi),  sp.sin(q3_sym)*sp.sin(pi), L3_mm*sp.cos(q3_sym)],
#     [sp.sin(q3_sym),  sp.cos(q3_sym)*sp.cos(pi), -sp.cos(q3_sym)*sp.sin(pi), L3_mm*sp.sin(q3_sym)],
#     [0,               sp.sin(pi),                sp.cos(pi),                0],
#     [0,               0,                        0,                         1]
# ])
#
# # Joint 3 to Joint 4 (this is Joint_5 in the robot)
# T34 = sp.Matrix([
#     [sp.cos(q4_sym), -sp.sin(q4_sym)*sp.cos(0),  sp.sin(q4_sym)*sp.sin(0), L4_mm*sp.cos(q4_sym)],
#     [sp.sin(q4_sym),  sp.cos(q4_sym)*sp.cos(0), -sp.cos(q4_sym)*sp.sin(0), L4_mm*sp.sin(q4_sym)],
#     [0,               sp.sin(0),                sp.cos(0),                0],
#     [0,               0,                        0,                        1]
# ])
#
# # Full transformation (base to EE)
# T04_sym = sp.simplify(T01 * T12 * T23 * T34)
#
# # Example to test (offline, not in ROS):
# # angle_values = {q1_sym: 0, q2_sym: sp.rad(90), q3_sym: 0, q4_sym: 0}  # if you want radians
# # T04_num = np.array(T04_sym.subs(angle_values).evalf(), dtype=float)
# # P04_mm = T04_num[0:3, 3]
# # print("End-effector (mm):", np.round(P04_mm, 4))


# ============================================================
# 2) NUMERIC FORWARD KINEMATICS (used in ROS)
#    Derived from the same DH model above.
# ============================================================

def forward_kinematics_func(q: np.ndarray) -> np.ndarray:
    """
    Input:
        q : np.array(4,) in RADIANS
            q[0] -> Joint_1 (q1)
            q[1] -> Joint_2 (q2)
            q[2] -> Joint_3 (q3)
            q[3] -> Joint_5 (q4 in the DH code)

    Output:
        X : np.array(3,) -> [x, y, z] in meters
        using the same DH model as the symbolic code above.
    """

    q = np.array(q, dtype=float).flatten()
    q1, q2, q3, q4 = q  # q4 corresponds to Joint_5

    # Link lengths (converted from mm to meters)
    L1 = 0.03855   # 38.55 mm
    L2 = 0.140     # 140  mm
    L3 = 0.103     # 103  mm
    L4 = 0.02985   # 29.85 mm

    # Radial reach in plane of q1
    R = L2 * math.sin(q2) \
        + L3 * math.sin(q2 - q3) \
        + L4 * math.sin(q2 - q3 + q4)

    x = -R * math.cos(q1)
    y = -R * math.sin(q1)
    z = L1 \
        + L2 * math.cos(q2) \
        + L3 * math.cos(q2 - q3) \
        + L4 * math.cos(q2 - q3 + q4)

    return np.array([x, y, z], dtype=float)


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
        "Joint_4": 0.0,   # ignored in math (always 0 in your tests)
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
