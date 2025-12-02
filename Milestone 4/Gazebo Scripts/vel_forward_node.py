#!/usr/bin/env python3
import rospy
import numpy as np
import math

# ============================================================
# 1) Forward Position Kinematics (same DH as before)
# ============================================================

def forward_kinematics_pos(q: np.ndarray) -> np.ndarray:
    """
    Input:
        q : np.array(4,) in RADIANS
            q[0] -> Joint_1 (q1)
            q[1] -> Joint_2 (q2)
            q[2] -> Joint_3 (q3)
            q[3] -> Joint_5 (q4)

    Output:
        X : np.array(3,) -> [x, y, z] (meters)
    """
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


# ============================================================
# 2) Helper: DH-based transforms to get z-axes for Jw
# ============================================================

def compute_T_matrices(q: np.ndarray):
    """
    Compute T01, T02, T03, T04 using the same DH model
    (simplified with pi/2, pi constants).
    """
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

    # T01 (simplified from MATLAB version)
    T01 = np.array([
        [ c1,  0.0,  s1,  0.0],
        [ s1,  0.0, -c1,  0.0],
        [0.0,  1.0, 0.0,  L1 ],
        [0.0,  0.0, 0.0,  1.0]
    ], dtype=float)

    # T12 (simplified from MATLAB)
    T12 = np.array([
        [ -s2,  c2, 0.0, -L2 * s2],
        [  c2,  s2, 0.0,  L2 * c2],
        [ 0.0, 0.0,-1.0,  0.0    ],
        [ 0.0, 0.0, 0.0,  1.0    ]
    ], dtype=float)

    # T23 (simplified from MATLAB)
    T23 = np.array([
        [ c3,  s3, 0.0, L3 * c3],
        [ s3, -c3, 0.0, L3 * s3],
        [0.0, 0.0,-1.0, 0.0    ],
        [0.0, 0.0, 0.0, 1.0    ]
    ], dtype=float)

    # T34 (simplified from MATLAB)
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


# ============================================================
# 3) Numerical Jacobian (linear part Jv)
# ============================================================

def numerical_jacobian_lin(q: np.ndarray, h: float = 1e-5) -> np.ndarray:
    """
    Numerical Jacobian of position X(q) = [x,y,z] wrt q (4x1).
    Returns Jv: 3x4
    """
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


# ============================================================
# 4) Full Geometric Jacobian J (6x4)
# ============================================================

def velocity_jacobian(q: np.ndarray) -> np.ndarray:
    """
    Returns 6x4 Jacobian:
        [Jv(q); Jw(q)]
    """
    q = np.array(q, dtype=float).flatten()

    # Linear part from numerical derivative of position
    Jv = numerical_jacobian_lin(q)

    # Angular part from joint axes z_{i-1} expressed in base frame
    T01, T02, T03, T04 = compute_T_matrices(q)

    z0 = np.array([0.0, 0.0, 1.0], dtype=float)
    z1 = T01[0:3, 2]
    z2 = T02[0:3, 2]
    z3 = T03[0:3, 2]

    Jw = np.column_stack([z0, z1, z2, z3])  # 3x4

    J = np.vstack([Jv, Jw])  # 6x4
    return J


# ============================================================
# 5) ROS Node: Forward Velocity
# ============================================================

def vel_forward_node():
    rospy.init_node("vel_forward_node", anonymous=True)
    rate = rospy.Rate(2)  # 2 Hz

    rospy.loginfo(
        "vel_forward_node started.\n"
        " - Reads Joint_1,2,3,5 (deg) and Joint_1_dot..Joint_5_dot (deg/s) from ROS params\n"
        " - Computes EE linear & angular velocity using J(q)*q_dot\n"
    )

    # Default angles and velocities in DEGREES / DEGREES PER SECOND
    defaults_deg = {
        "Joint_1": 30.0,
        "Joint_2": 30.0,
        "Joint_3": -30.0,
        "Joint_4": 0.0,  # not used
        "Joint_5": 30.0,
    }

    defaults_deg_dot = {
        "Joint_1_dot": 5.0,
        "Joint_2_dot": 5.0,
        "Joint_3_dot": 5.0,
        "Joint_4_dot": 0.0,
        "Joint_5_dot": 5.0,
    }

    while not rospy.is_shutdown():
        # read joint angles (deg)
        J1_deg = float(rospy.get_param("Joint_1", defaults_deg["Joint_1"]))
        J2_deg = float(rospy.get_param("Joint_2", defaults_deg["Joint_2"]))
        J3_deg = float(rospy.get_param("Joint_3", defaults_deg["Joint_3"]))
        J5_deg = float(rospy.get_param("Joint_5", defaults_deg["Joint_5"]))

        q_deg = np.array([J1_deg, J2_deg, J3_deg, J5_deg], dtype=float)
        q_rad = np.deg2rad(q_deg)

        # read joint velocities (deg/s)
        J1d_deg = float(rospy.get_param("Joint_1_dot", defaults_deg_dot["Joint_1_dot"]))
        J2d_deg = float(rospy.get_param("Joint_2_dot", defaults_deg_dot["Joint_2_dot"]))
        J3d_deg = float(rospy.get_param("Joint_3_dot", defaults_deg_dot["Joint_3_dot"]))
        J5d_deg = float(rospy.get_param("Joint_5_dot", defaults_deg_dot["Joint_5_dot"]))

        qd_deg = np.array([J1d_deg, J2d_deg, J3d_deg, J5d_deg], dtype=float)
        qd_rad = np.deg2rad(qd_deg)  # rad/s

        # compute Jacobian and EE twist
        J = velocity_jacobian(q_rad)
        twist = J @ qd_rad

        vx, vy, vz, wx, wy, wz = twist  # [m/s, m/s, m/s, rad/s, rad/s, rad/s]

        rospy.loginfo_throttle(
            0.5,
            f"[Vel-FK] q(deg) = [J1={J1_deg:.2f}, J2={J2_deg:.2f}, J3={J3_deg:.2f}, J5={J5_deg:.2f}], "
            f"qd(deg/s) = [J1d={J1d_deg:.2f}, J2d={J2d_deg:.2f}, J3d={J3d_deg:.2f}, J5d={J5d_deg:.2f}]\n"
            f"         EE linear vel  v = [{vx:.4f}, {vy:.4f}, {vz:.4f}] m/s\n"
            f"         EE angular vel w = [{wx:.4f}, {wy:.4f}, {wz:.4f}] rad/s"
        )

        rate.sleep()


if __name__ == "__main__":
    vel_forward_node()
