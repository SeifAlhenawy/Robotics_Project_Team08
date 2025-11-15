#!/usr/bin/env python3
# ee_listener.py
# Reads the end-effector position (EE_frame) from /gazebo/link_states.

import rospy
from gazebo_msgs.msg import LinkStates

def main():
    rospy.init_node("ee_listener", anonymous=True)
    EE_LINK_NAME = "my_robot::EE_frame"   #  correct link name from /gazebo/link_states
    rospy.loginfo("EE listener node started (looking for link: %s)", EE_LINK_NAME)

    state = {"found": False}

    def callback(msg: LinkStates):
        try:
            i = msg.name.index(EE_LINK_NAME)
        except ValueError:
            rospy.logwarn_throttle(2.0, "Link '%s' not yet found in /gazebo/link_states", EE_LINK_NAME)
            return

        p = msg.pose[i].position
        if not state["found"]:
            rospy.loginfo(" Found link '%s' in /gazebo/link_states", EE_LINK_NAME)
            state["found"] = True

        rospy.loginfo_throttle(0.5, "EE position → x=%.4f  y=%.4f  z=%.4f", p.x, p.y, p.z)

    rospy.Subscriber("/gazebo/link_states", LinkStates, callback, queue_size=10)
    rospy.spin()

if __name__ == "__main__":
    main()

