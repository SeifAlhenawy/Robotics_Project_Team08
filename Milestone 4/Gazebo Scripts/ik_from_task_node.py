#!/usr/bin/env python3
import rospy
import numpy as np
import math

# ============================================================
# 1) FORWARD KINEMATICS (SAME URDF-BASED MODEL AS YOUR FK NODE)
#    World -> Rotating_Waste -> Arm_1 -> Arm_2 -> Arm_3 -> Gripper -> EE_frame
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
        for the link **EE_frame**.
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
    #  URDF CHAIN
    # ============================================================

    # 0) World → Joint_1 (Rotating_Waste)
    T0_1 = T(RotZ(q1), np.array([0.0, 0.0, 0.050]))  # approx base height 0.05 m

    # 1) Joint_1 → Joint_2
    origin_J2 = np.array([-2.7567e-05, 0.0061999, 0.04975])
    R_J2_fixed = RotZ(0.0) @ RotY(0.0) @ RotX(1.5708)
    R_J2 = R_J2_fixed @ RotZ(q2)  # axis="0 0 1"
    T1_2 = T(R_J2, origin_J2)

    # 2) Joint_2 → Joint_3
    origin_J3 = np.array([0.0, 0.140, 0.0])
    R_J3_fixed = RotZ(1.5708)
    R_J3 = R_J3_fixed @ RotZ(-q3)  # axis="0 0 -1"
    T2_3 = T(R_J3, origin_J3)

    # 3) Joint_3 → Joint_4 (Joint_4 = 0 always, fixed rot)
    R_J4_fixed = RotZ(-1.5708) @ RotY(0.0) @ RotX(-1.5708)
    T3_4 = T(R_J4_fixed, np.array([0.0, 0.0, 0.0]))

    # 4) Joint_4 → Joint_5
    origin_J5 = np.array([-0.001651, -0.0054574, 0.13289])
    R_J5_fixed = RotZ(0.0) @ RotY(0.0) @ RotX(1.5708)
    R_J5 = R_J5_fixed @ RotZ(q4)  # axis="0 0 1"
    T4_5 = T(R_J5, origin_J5)

    # 5) Gripper → EE_frame (EE_frame_lock fixed joint)
    # TODO: replace these with the exact values from your URDF if needed.
    x_EE = 0.0
    y_EE = 0.0
    z_EE = 0.10
    roll_EE  = 0.0
    pitch_EE = 0.0
    yaw_EE   = 0.0

    origin_EE = np.array([x_EE, y_EE, z_EE])
    R_EE_fixed = RotZ(yaw_EE) @ RotY(pitch_EE) @ RotX(roll_EE)
    T5_EE = T(R_EE_fixed, origin_EE)

    T0_EE = T0_1 @ T1_2 @ T2_3 @ T3_4 @ T4_5 @ T5_EE
    pos = T0_EE[0:3, 3]
    return pos


# ============================================================
# 2) NUMERICAL JACOBIAN f(q) = X(q) (3x1)  ->  J = dX/dq
# ============================================================

def numerical_jacobian(q: np.ndarray, h: float = 1e-5) -> np.ndarray:
    q = np.array(q, dtype=float).flatten()
    n = q.size
    J = np.zeros((3, n), dtype=float)

    for j in range(n):
        dq = np.zeros_like(q)
        dq[j] = h
        X_plus = forward_kinematics_func(q + dq)
        X_minus = forward_kinematics_func(q - dq)
        J[:, j] = (X_plus - X_minus) / (2.0 * h)

    return J


# ============================================================
# 3) HELPER: ANGLE WRAP & JOINT LIMITS
# ============================================================

def wrap_to_pi(angle_rad: float) -> float:
    return (angle_rad + np.pi) % (2.0 * np.pi) - np.pi

def apply_joint_limits(q: np.ndarray) -> np.ndarray:
    """
    Apply joint limits in radians.
    >>>>>> TUNE THESE to match your URDF if needed <<<<<<
    """

    q = np.array(q, dtype=float).flatten()

    # Reasonable safe limits (you can tighten them further):
    #   J1: base rotation, ±120°
    #   J2: shoulder,   -20° .. +100°  (avoid pointing fully backwards)
    #   J3: elbow,      -120° .. +10°
    #   J5: wrist,      -90° .. +90°
    limits_deg = [
        (-120.0, 120.0),   # Joint_1
        (-20.0, 100.0),    # Joint_2
        (-120.0, 10.0),    # Joint_3
        (-90.0, 90.0)      # Joint_5
    ]

    limits_rad = [(np.deg2rad(lo), np.deg2rad(hi)) for (lo, hi) in limits_deg]

    for i in range(4):
        lo, hi = limits_rad[i]
        # wrap to [-pi, pi] then clamp to [lo, hi]
        q[i] = (q[i] + np.pi) % (2.0 * np.pi) - np.pi
        q[i] = np.clip(q[i], lo, hi)

    return q


# ============================================================
# 4) DAMPED LEVENBERG–MARQUARDT IK SOLVER
# ============================================================

def ik_solve_damped(q_init: np.ndarray,
                    X_des: np.ndarray,
                    max_iters: int = 200,
                    tol: float = 2e-2):# 0.02 m = 2 cm
    """
    Damped Least Squares (Levenberg–Marquardt) inverse kinematics:

        error = X_des - X(q)
        J = dX/dq
        dq = Jᵀ (J Jᵀ + λ² I)⁻¹ error

    Adaptive λ similar to your friend's MATLAB function.
    """

    q = np.array(q_init, dtype=float).flatten()
    X_des = np.array(X_des, dtype=float).flatten()

    lam = 0.01        # initial damping
    lam_min = 1e-5
    lam_max = 0.1

    for k in range(max_iters):
        X_cur = forward_kinematics_func(q)
        error_vec = X_des - X_cur
        err_norm = np.linalg.norm(error_vec)

        rospy.loginfo(
            f"[IK] Iter {k:03d} | err = {err_norm:.6e} | "
            f"lambda = {lam:.3e} | q(deg) = {np.rad2deg(q)}"
        )

        # Stopping condition based on error
        if err_norm < tol:
            rospy.loginfo(
                "-------------------------------\n"
                "IK SOLUTION SUCCESS (Damped LS)\n"
                f"Target XYZ = [{X_des[0]:.3f}, {X_des[1]:.3f}, {X_des[2]:.3f}] m\n"
                f"Solution (deg) = {np.rad2deg(q)}\n"
                f"Error = {err_norm:.6e} m | Iter = {k}\n"
                "-------------------------------"
            )
            return q, err_norm, k

        # Jacobian
        J = numerical_jacobian(q)

        # Condition number & singular values
        try:
            cond_J = np.linalg.cond(J)
        except np.linalg.LinAlgError:
            cond_J = np.inf
        s = np.linalg.svd(J, compute_uv=False)
        min_sv = np.min(s)

        rospy.loginfo(f"[IK]    cond(J) = {cond_J:.3e} | min_sv = {min_sv:.3e}")

        # Adaptive damping
        if min_sv < 1e-3:
            lam = min(lam_max, lam * 2.0)      # more damping near singularity
        else:
            lam = max(lam_min, lam / 1.5)      # reduce damping if well-conditioned

        # Damped least-squares pseudo-inverse
        JJt = J @ J.T      # 3x3
        lam_I = (lam**2) * np.eye(3)
        try:
            pinv_dls = J.T @ np.linalg.inv(JJt + lam_I)  # 4x3
        except np.linalg.LinAlgError:
            rospy.logwarn("[IK] Matrix inversion failed in DLS; aborting.")
            break

        dq = pinv_dls @ error_vec  # 4x1
        q = q + dq                  # NEXT iteration starts from this q
        q = apply_joint_limits(q)   # keep within joint limits
        
    # If we are here, not converged within max_iters
    X_cur = forward_kinematics_func(q)
    error_vec = X_des - X_cur
    err_norm = np.linalg.norm(error_vec)

    rospy.logwarn(
        "-------------------------------\n"
        "[IK] DID NOT REACH TOL, USING BEST FOUND q\n"
        f"Target XYZ = [{X_des[0]:.3f}, {X_des[1]:.3f}, {X_des[2]:.3f}] m\n"
        f"Last q(deg) = {np.rad2deg(q)}\n"
        f"Final error = {err_norm:.6e} m | Iter = {max_iters}\n"
        "-------------------------------"
    )
    # IMPORTANT: return q, NOT None
    return q, err_norm, max_iters


def ik_node():
    rospy.init_node("ik_from_task_node", anonymous=True)
    rate = rospy.Rate(0.2)  # every 5 sec (just for testing; adjust for trajectory)

    rospy.loginfo(
        "IK Optimal Node Started (Damped LS, 200 iterations)\n"
        " - Reads desired EE position from params: IK_X_des, IK_Y_des, IK_Z_des (meters)\n"
        " - Uses current Joint_1,2,3,5 (deg) as initial guess\n"
        " - Solves IK via adaptive Damped LS\n"
        " - Behavior controlled by param: IK_ENABLE_CONTROL (bool)\n"
        "     * True  -> writes Joint_* (controls Gazebo)\n"
        "     * False -> math-only, writes IK_J*_sol, does NOT touch Joint_*\n"
    )

    defaults_des = {
        "IK_X_des": 0.10,
        "IK_Y_des": 0.05,
        "IK_Z_des": 0.20,
    }

    while not rospy.is_shutdown():
        # 0) Check mode (controller vs math-only)
        enable_control = bool(rospy.get_param("IK_ENABLE_CONTROL", True))

        # 1) Desired task-space position
        Xd = float(rospy.get_param("IK_X_des", defaults_des["IK_X_des"]))
        Yd = float(rospy.get_param("IK_Y_des", defaults_des["IK_Y_des"]))
        Zd = float(rospy.get_param("IK_Z_des", defaults_des["IK_Z_des"]))
        X_des = np.array([Xd, Yd, Zd], dtype=float)

        # 2) Current joints as initial guess (deg → rad)
        J1_deg = float(rospy.get_param("Joint_1", 30.0))
        J2_deg = float(rospy.get_param("Joint_2", 30.0))
        J3_deg = float(rospy.get_param("Joint_3", -30.0))
        J5_deg = float(rospy.get_param("Joint_5", 30.0))

        q_init_deg = np.array([J1_deg, J2_deg, J3_deg, J5_deg], dtype=float)
        q_init_rad = np.deg2rad(q_init_deg)
        q_init_rad = apply_joint_limits(q_init_rad)

        # 3) Run IK
        q_sol_rad, err, iters = ik_solve_damped(
            q_init_rad,
            X_des,
            max_iters=200,
            tol=1e-4
        )

        if q_sol_rad is None:
            rospy.logwarn_throttle(
                5.0,
                f"[IK] Failed to converge: final error = {err:.6f} m after {iters} iterations."
            )
        else:
            q_sol_deg = np.rad2deg(q_sol_rad)
            J1_s, J2_s, J3_s, J5_s = q_sol_deg

            # Always log
            rospy.loginfo_throttle(
                5.0,
                f"[IK] Target Xd = [{Xd:.4f}, {Yd:.4f}, {Zd:.4f}] m\n"
                f"     Initial guess (deg)  = [J1={J1_deg:.2f}, J2={J2_deg:.2f}, "
                f"J3={J3_deg:.2f}, J5={J5_deg:.2f}]\n"
                f"     Solution (deg)       = [J1={J1_s:.2f}, J2={J2_s:.2f}, "
                f"J3={J3_s:.2f}, J5={J5_s:.2f}]\n"
                f"     Final error ||f||    = {err:.6e} m in {iters} iterations."
            )

            # 3a) Math-only outputs (always, for debugging/trajectory use)
            rospy.set_param("IK_J1_sol", float(J1_s))
            rospy.set_param("IK_J2_sol", float(J2_s))
            rospy.set_param("IK_J3_sol", float(J3_s))
            rospy.set_param("IK_J4_sol", 0.0)
            rospy.set_param("IK_J5_sol", float(J5_s))

            # 3b) Optional: control the robot (only if enabled)
            if enable_control:
                rospy.set_param("Joint_1", float(J1_s))
                rospy.set_param("Joint_2", float(J2_s))
                rospy.set_param("Joint_3", float(J3_s))
                rospy.set_param("Joint_4", 0.0)
                rospy.set_param("Joint_5", float(J5_s))

        rate.sleep()


if __name__ == "__main__":
    try:
        ik_node()
    except rospy.ROSInterruptException:
        pass
