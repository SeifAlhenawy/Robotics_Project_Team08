#!/usr/bin/env python3
import rospy
import numpy as np
import math

# -------------------------------------
# FORWARD KINEMATICS (same as existing)
# -------------------------------------
def forward_kinematics_func(q: np.ndarray) -> np.ndarray:
    q = np.array(q, dtype=float).flatten()
    q1, q2, q3, q4 = q

    def RotX(a):
        ca, sa = np.cos(a), np.sin(a)
        return np.array([[1, 0, 0],
                         [0, ca, -sa],
                         [0, sa,  ca]])

    def RotY(a):
        ca, sa = np.cos(a), np.sin(a)
        return np.array([[ ca, 0, sa],
                         [  0, 1,  0],
                         [-sa, 0, ca]])

    def RotZ(a):
        ca, sa = np.cos(a), np.sin(a)
        return np.array([[ca, -sa, 0],
                         [sa,  ca, 0],
                         [ 0,   0, 1]])

    def T(R, p):
        Tmat = np.eye(4)
        Tmat[0:3, 0:3] = R
        Tmat[0:3, 3]   = p
        return Tmat

    T0_1 = T(RotZ(q1), np.array([0.0, 0.0, 0.050]))

    origin_J2   = np.array([-2.7567e-05, 0.0061999, 0.04975])
    R_J2_fixed  = RotX(1.5708)
    R_J2        = R_J2_fixed @ RotZ(q2)
    T1_2        = T(R_J2, origin_J2)

    origin_J3   = np.array([0.0, 0.140, 0.0])
    R_J3_fixed  = RotZ(1.5708)
    R_J3        = R_J3_fixed @ RotZ(-q3)
    T2_3        = T(R_J3, origin_J3)

    R_J4_fixed  = RotZ(-1.5708) @ RotX(-1.5708)
    T3_4        = T(R_J4_fixed, np.array([0.0, 0.0, 0.0]))

    origin_J5   = np.array([-0.001651, -0.0054574, 0.13289])
    R_J5_fixed  = RotX(1.5708)
    R_J5        = R_J5_fixed @ RotZ(q4)
    T4_5        = T(R_J5, origin_J5)

    T5_EE       = T(np.eye(3), np.array([0.0, 0.0, 0.10]))
    T0_EE       = T0_1 @ T1_2 @ T2_3 @ T3_4 @ T4_5 @ T5_EE

    return T0_EE[0:3, 3]


# -------------------------------------
# CIRCULAR TRAJECTORY GENERATOR
# -------------------------------------
def task_traj_circle(center, radius, Tf, Ts, direction=1):
    center = np.array(center, dtype=float).flatten()
    Cx, Cy, Cz = center

    t_vec = np.arange(0.0, Tf + Ts, Ts)
    Task = np.zeros((len(t_vec), 3))

    for i, t in enumerate(t_vec):
        theta = direction * 2.0 * np.pi * (t / Tf)
        x = Cx + radius * math.cos(theta)
        y = Cy + radius * math.sin(theta)
        z = Cz
        Task[i, :] = [x, y, z]

    return t_vec, Task


# -------------------------------------
# ROS NODE
# -------------------------------------
def main():
    rospy.init_node("traj_circle_node")

    Ts = rospy.get_param("TRAJ_Ts", 0.1)
    Tf = rospy.get_param("TRAJ_Tf", 16.0)

    Cx = rospy.get_param("TRAJ_CX", 0.18)
    Cy = rospy.get_param("TRAJ_CY", 0.00)
    Cz = rospy.get_param("TRAJ_CZ", 0.14)
    R  = rospy.get_param("TRAJ_R",  0.03)
    direction = int(rospy.get_param("TRAJ_DIR", 1))

    rospy.loginfo("=== CIRCULAR TASK TRAJECTORY ===")
    rospy.loginfo("Center = [%.3f, %.3f, %.3f], R=%.3f, direction=%d", Cx, Cy, Cz, R, direction)
    rospy.loginfo("Ts = %.3f  Tf = %.3f", Ts, Tf)
    rospy.loginfo("Make sure IK_ENABLE_CONTROL is TRUE before running trajectory.")

    t_vec, TaskSpace = task_traj_circle([Cx, Cy, Cz], R, Tf, Ts, direction)
    rate = rospy.Rate(1.0 / Ts)

    for k, t in enumerate(t_vec):
        if rospy.is_shutdown(): break

        X_des = TaskSpace[k, :]

        rospy.set_param("IK_X_des", float(X_des[0]))
        rospy.set_param("IK_Y_des", float(X_des[1]))
        rospy.set_param("IK_Z_des", float(X_des[2]))

        rospy.sleep(0.25)

        J1 = float(rospy.get_param("Joint_1", 0.0))
        J2 = float(rospy.get_param("Joint_2", 0.0))
        J3 = float(rospy.get_param("Joint_3", 0.0))
        J5 = float(rospy.get_param("Joint_5", 0.0))

        q_deg = np.array([J1, J2, J3, J5], float)
        q_rad = np.deg2rad(q_deg)
        X_fk  = forward_kinematics_func(q_rad)

        rospy.loginfo("k=%03d | Xdes=[%.4f %.4f %.4f] | q(deg)=[%.2f %.2f %.2f %.2f] | FK=[%.4f %.4f %.4f]",
                      k, X_des[0], X_des[1], X_des[2],
                      J1, J2, J3, J5,
                      X_fk[0], X_fk[1], X_fk[2])

        rate.sleep()

    rospy.loginfo("=== Circular trajectory finished ===")


if __name__ == "__main__":
    try: main()
    except rospy.ROSInterruptException: pass
