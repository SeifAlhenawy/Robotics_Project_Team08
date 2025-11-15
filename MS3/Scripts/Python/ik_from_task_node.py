#!/usr/bin/env python3
import rospy
import numpy as np
import math

# ============================================================
# 1) Forward Kinematics (same DH model as your MATLAB code)
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
        using the same DH model as your MATLAB script.
    """

    q = np.array(q, dtype=float).flatten()
    q1, q2, q3, q4 = q  # q4 corresponds to Joint_5

    # Link lengths (mm -> m)
    L1 = 0.03855   # 38.55 mm
    L2 = 0.140     # 140   mm
    L3 = 0.103     # 103   mm
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
# 2) Numerical Jacobian (approximate f_dot = ∂f/∂q)
#    f(q) = X(q) - X_des  (3x1)
# ============================================================

def numerical_jacobian(q: np.ndarray, h: float = 1e-5) -> np.ndarray:
    """
    Compute numerical Jacobian of f(q) = X(q) around q using
    central differences.

    J_ij = ∂f_i / ∂q_j ≈ (f_i(q + h e_j) - f_i(q - h e_j)) / (2h)
    """
    q = np.array(q, dtype=float).flatten()
    n = q.size
    J = np.zeros((3, n), dtype=float)

    for j in range(n):
        dq = np.zeros_like(q)
        dq[j] = h

        X_plus = forward_kinematics_func(q + dq)
        X_minus = forward_kinematics_func(q - dq)

        # f(q) = X(q) - X_des -> derivative same as X(q)
        J[:, j] = (X_plus - X_minus) / (2.0 * h)

    return J


# ============================================================
# 3) Newton–Raphson IK solver (like your MATLAB loop)
# ============================================================

def ik_solve(q_init: np.ndarray,
             X_des: np.ndarray,
             max_iters: int = 20,
             error_threshold: float = 1e-3):
    """
    Newton–Raphson inverse kinematics:

        f(q) = X(q) - X_des   (3x1)
        J(q) = ∂f/∂q
        q_{k+1} = q_k - J^+ f(q_k)

    Inputs:
        q_init         : np.array(4,) initial guess [rad]
        X_des          : np.array(3,) desired [x,y,z] in m
        max_iters      : maximum iterations
        error_threshold: stopping threshold on ||f||

    Returns:
        q_sol : np.array(4,) in rad (or None if fail)
        err   : final error norm
        iters : number of iterations used
    """

    q = np.array(q_init, dtype=float).flatten()
    X_des = np.array(X_des, dtype=float).flatten()

    for k in range(max_iters):
        X = forward_kinematics_func(q)
        f = X - X_des  # f(q) = X(q) - X_des
        err = np.linalg.norm(f)

        if err < error_threshold:
            return q, err, (k + 1)

        J = numerical_jacobian(q)

        # Use pseudo-inverse for robustness (like pinv in MATLAB)
        J_pinv = np.linalg.pinv(J)

        dq = J_pinv @ f
        q = q - dq

    # If we exit the loop without meeting error_threshold:
    X = forward_kinematics_func(q)
    f = X - X_des
    err = np.linalg.norm(f)
    return None, err, max_iters


# ============================================================
# 4) ROS node: read desired X,Y,Z and current joints, solve IK
# ============================================================

def ik_node():
    rospy.init_node("ik_from_task_node", anonymous=True)
    rate = rospy.Rate(0.2)  # solve at 0.2 Hz (every 5s) just for testing

    rospy.loginfo(
        "ik_from_task_node started.\n"
        " - Reads desired EE position from params: IK_X_des, IK_Y_des, IK_Z_des (meters)\n"
        " - Uses current Joint_1,2,3,5 (deg) as initial guess\n"
        " - Solves IK and prints joint solution (deg)\n"
        " - Optionally writes solution back to Joint_1..Joint_5 params."
    )

    # Default desired position (in meters)
    # You can change them using rosparam set IK_X_des 0.1 etc.
    defaults_des = {
        "IK_X_des": 0.10,  # 10 cm
        "IK_Y_des": 0.05,  # 5  cm
        "IK_Z_des": 0.20,  # 20 cm
    }

    while not rospy.is_shutdown():
        # 1) Read desired task-space position (meters)
        Xd = float(rospy.get_param("IK_X_des", defaults_des["IK_X_des"]))
        Yd = float(rospy.get_param("IK_Y_des", defaults_des["IK_Y_des"]))
        Zd = float(rospy.get_param("IK_Z_des", defaults_des["IK_Z_des"]))
        X_des = np.array([Xd, Yd, Zd], dtype=float)

        # 2) Read current joints as initial guess (degrees)
        #    We use the same params as your joint_publisher_param.py.
        J1_deg = float(rospy.get_param("Joint_1", 30.0))
        J2_deg = float(rospy.get_param("Joint_2", 30.0))
        J3_deg = float(rospy.get_param("Joint_3", -30.0))
        J5_deg = float(rospy.get_param("Joint_5", 30.0))

        q_init_deg = np.array([J1_deg, J2_deg, J3_deg, J5_deg], dtype=float)
        q_init_rad = np.deg2rad(q_init_deg)

        # 3) Run IK
        q_sol_rad, err, iters = ik_solve(
            q_init_rad,
            X_des,
            max_iters=20,
            error_threshold=1e-3
        )

        if q_sol_rad is None:
            rospy.logwarn_throttle(
                5.0,
                f"[IK] Failed to converge: final error = {err:.6f} m after {iters} iterations."
            )
        else:
            q_sol_deg = np.rad2deg(q_sol_rad)
            J1_s, J2_s, J3_s, J5_s = q_sol_deg

            rospy.loginfo_throttle(
                5.0,
                f"[IK] Target Xd = [{Xd:.4f}, {Yd:.4f}, {Zd:.4f}] m\n"
                f"     Initial guess (deg)  = [J1={J1_deg:.2f}, J2={J2_deg:.2f}, "
                f"J3={J3_deg:.2f}, J5={J5_deg:.2f}]\n"
                f"     Solution (deg)       = [J1={J1_s:.2f}, J2={J2_s:.2f}, "
                f"J3={J3_s:.2f}, J5={J5_s:.2f}]\n"
                f"     Final error ||f||    = {err:.6e} m in {iters} iterations."
            )

            # 4) OPTIONAL: write solution back to joint params
            #    So your joint_publisher_param.py can move the arm to this pose.
            rospy.set_param("Joint_1", float(J1_s))
            rospy.set_param("Joint_2", float(J2_s))
            rospy.set_param("Joint_3", float(J3_s))
            # Joint_4 remains 0
            rospy.set_param("Joint_4", 0.0)
            rospy.set_param("Joint_5", float(J5_s))

        rate.sleep()


if __name__ == "__main__":
    ik_node()
