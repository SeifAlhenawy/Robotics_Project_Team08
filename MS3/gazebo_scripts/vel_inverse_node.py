#!/usr/bin/env python3
import rospy
import numpy as np
import math

# ---------- Same FK and Jacobian helpers as in vel_forward_node ----------

def forward_kinematics_pos(q: np.ndarray) -> np.ndarray:
    q = np.array(q, dtype=float).flatten()
    q1, q2, q3, q4 = q

    L1 = 0.03855
    L2 = 0.140
    L3 = 0.103
    L4 = 0.02985

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


def compute_T_matrices(q: np.ndarray):
    q = np.array(q, dtype=float).flatten()
    q1, q2, q3, q4 = q

    L1 = 0.03855
    L2 = 0.140
    L3 = 0.103
    L4 = 0.02985

    c1, s1 = math.cos(q1), math.sin(q1)
    c2, s2 = math.cos(q2), math.sin(q2)
    c3, s3 = math.cos(q3), math.sin(q3)
    c4, s4 = math.cos(q4), math.sin(q4)

    T01 = np.array([
        [ c1,  0.0,  s1,  0.0],
        [ s1,  0.0, -c1,  0.0],
        [0.0,  1.0, 0.0,  L1 ],
        [0.0,  0.0, 0.0,  1.0]
    ], dtype=float)

    T12 = np.array([
        [ -s2,  c2, 0.0, -L2 * s2],
        [  c2,  s2, 0.0,  L2 * c2],
        [ 0.0, 0.0,-1.0,  0.0    ],
        [ 0.0, 0.0, 0.0,  1.0    ]
    ], dtype=float)

    T23 = np.array([
        [ c3,  s3, 0.0, L3 * c3],
        [ s3, -c3, 0.0, L3 * s3],
        [0.0, 0.0,-1.0, 0.0    ],
        [0.0, 0.0, 0.0, 1.0    ]
    ], dtype=float)

    T34 = np.array([
        [ c4, -s4, 0.0, L4 * c4],
        [ s4,  c4, 0.0, L4 * s4],
        [0.0, 0.0, 1.0, 0.0    ],
        [0.0, 0.0, 0.0, 1.0    ]
    ], dtype=float)

    T02 = T01 @ T12
    T03 = T02 @ T23
    T04 = T03 @ T34

    return T01, T02, T03, T04


def numerical_jacobian_lin(q: np.ndarray, h: float = 1e-5) -> np.ndarray:
    q = np.array(q, dtype=float).flatten()
    n = q.size
    Jv = np.zeros((3, n), dtype=float)

    for j in range(n):
        dq = np.zeros_like(q)
        dq[j] = h
        X_plus = forward_kinematics_pos(q + dq)
        X_minus = forward_kinematics_pos(q - dq)
        Jv[:, j] = (X_plus - X_minus) / (2.0 * h)

    return Jv


def velocity_jacobian(q: np.ndarray) -> np.ndarray:
    q = np.array(q, dtype=float).flatten()
    Jv = numerical_jacobian_lin(q)

    T01, T02, T03, T04 = compute_T_matrices(q)
    z0 = np.array([0.0, 0.0, 1.0], dtype=float)
    z1 = T01[0:3, 2]
    z2 = T02[0:3, 2]
    z3 = T03[0:3, 2]

    Jw = np.column_stack([z0, z1, z2, z3])
    J = np.vstack([Jv, Jw])
    return J


# ============================================================
# 5) ROS Node: Inverse Velocity
# ============================================================

def vel_inverse_node():
    rospy.init_node("vel_inverse_node", anonymous=True)
    rate = rospy.Rate(0.5)  # 0.5 Hz

    rospy.loginfo(
        "vel_inverse_node started.\n"
        " - Reads current Joint_1,2,3,5 (deg) as configuration\n"
        " - Reads desired EE twist from params: "
        "VEL_VX_des, VEL_VY_des, VEL_VZ_des [m/s], "
        "VEL_WX_des, VEL_WY_des, VEL_WZ_des [rad/s]\n"
        " - Solves q_dot = J^+ * X_dot_des (pseudo-inverse)\n"
        " - Prints joint velocities (deg/s) and writes them to Joint_*_dot."
    )

    # Default desired EE velocities
    defaults_vel = {
        "VEL_VX_des": 0.05,   # 5 cm/s in x
        "VEL_VY_des": 0.00,
        "VEL_VZ_des": 0.00,
        "VEL_WX_des": 0.00,
        "VEL_WY_des": 0.00,
        "VEL_WZ_des": 0.00,
    }

    while not rospy.is_shutdown():
        # 1) Read current joint angles (deg)
        J1_deg = float(rospy.get_param("Joint_1", 30.0))
        J2_deg = float(rospy.get_param("Joint_2", 30.0))
        J3_deg = float(rospy.get_param("Joint_3", -30.0))
        J5_deg = float(rospy.get_param("Joint_5", 30.0))

        q_deg = np.array([J1_deg, J2_deg, J3_deg, J5_deg], dtype=float)
        q_rad = np.deg2rad(q_deg)

        # 2) Read desired EE twist (v, w)
        Vx = float(rospy.get_param("VEL_VX_des", defaults_vel["VEL_VX_des"]))
        Vy = float(rospy.get_param("VEL_VY_des", defaults_vel["VEL_VY_des"]))
        Vz = float(rospy.get_param("VEL_VZ_des", defaults_vel["VEL_VZ_des"]))
        Wx = float(rospy.get_param("VEL_WX_des", defaults_vel["VEL_WX_des"]))
        Wy = float(rospy.get_param("VEL_WY_des", defaults_vel["VEL_WY_des"]))
        Wz = float(rospy.get_param("VEL_WZ_des", defaults_vel["VEL_WZ_des"]))

        Xdot_des = np.array([Vx, Vy, Vz, Wx, Wy, Wz], dtype=float)  # [m/s, rad/s]

        # 3) Build Jacobian and solve q_dot = J^+ Xdot
        J = velocity_jacobian(q_rad)  # 6x4

        J_pinv = np.linalg.pinv(J)    # 4x6
        qdot_rad = J_pinv @ Xdot_des  # 4x1 rad/s
        qdot_deg = np.rad2deg(qdot_rad)  # deg/s

        J1d, J2d, J3d, J5d = qdot_deg

        rospy.loginfo_throttle(
            2.0,
            f"[Vel-IK] q(deg) = [J1={J1_deg:.2f}, J2={J2_deg:.2f}, J3={J3_deg:.2f}, J5={J5_deg:.2f}]\n"
            f"         Xdot_des = v=[{Vx:.4f}, {Vy:.4f}, {Vz:.4f}] m/s, "
            f"w=[{Wx:.4f}, {Wy:.4f}, {Wz:.4f}] rad/s\n"
            f"         qdot_sol (deg/s) = [J1d={J1d:.2f}, J2d={J2d:.2f}, J3d={J3d:.2f}, J5d={J5d:.2f}]"
        )

        # 4) Optionally write back to Joint_i_dot params
        rospy.set_param("Joint_1_dot", float(J1d))
        rospy.set_param("Joint_2_dot", float(J2d))
        rospy.set_param("Joint_3_dot", float(J3d))
        rospy.set_param("Joint_4_dot", 0.0)
        rospy.set_param("Joint_5_dot", float(J5d))

        rate.sleep()


if __name__ == "__main__":
    vel_inverse_node()
