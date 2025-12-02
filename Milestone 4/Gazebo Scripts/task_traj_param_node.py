#!/usr/bin/env python3
import rospy
import numpy as np
import math

def main():
    rospy.init_node("task_traj_simple_node")

    # ===== USER PARAMS (change using rosparam set) =====
    Ts = rospy.get_param("TRAJ_Ts", 0.2)    # sampling time (slow and safe)
    Tf = rospy.get_param("TRAJ_Tf", 12.0)   # total time

    X0 = np.array([
        float(rospy.get_param("TRAJ_X0", 0.16)),
        float(rospy.get_param("TRAJ_Y0", 0.00)),
        float(rospy.get_param("TRAJ_Z0", 0.10)),
    ])

    Xf = np.array([
        float(rospy.get_param("TRAJ_Xf", 0.16)),
        float(rospy.get_param("TRAJ_Yf", 0.00)),
        float(rospy.get_param("TRAJ_Zf", 0.18)),
    ])

    rospy.loginfo("=== SIMPLE STRAIGHT TASK TRAJECTORY ===")
    rospy.loginfo("Ts=%.2f Tf=%.2f", Ts, Tf)
    rospy.loginfo("Start X0 = %s", X0)
    rospy.loginfo("Final Xf = %s", Xf)
    rospy.loginfo("Make sure ik_from_task_node.py and joint_publisher_param.py are running!")

    t_vec = np.arange(0.0, Tf + Ts, Ts)
    rate = rospy.Rate(1.0 / Ts)

    for k, t in enumerate(t_vec):
        if rospy.is_shutdown():
            break

        s = t / Tf
        X_des = X0 + s * (Xf - X0)

        rospy.set_param("IK_X_des", float(X_des[0]))
        rospy.set_param("IK_Y_des", float(X_des[1]))
        rospy.set_param("IK_Z_des", float(X_des[2]))

        rospy.sleep(0.25)

        J1 = float(rospy.get_param("Joint_1", 0.0))
        J2 = float(rospy.get_param("Joint_2", 0.0))
        J3 = float(rospy.get_param("Joint_3", 0.0))
        J5 = float(rospy.get_param("Joint_5", 0.0))

        rospy.loginfo(
            "k=%02d | X_des=[%.3f %.3f %.3f] | q(deg)=[%.2f %.2f %.2f %.2f]",
            k, X_des[0], X_des[1], X_des[2], J1, J2, J3, J5
        )

        rate.sleep()

    rospy.loginfo("===== TRAJECTORY FINISHED =====")


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
