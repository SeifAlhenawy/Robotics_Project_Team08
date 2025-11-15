#!/usr/bin/env python3
import rospy, math # type: ignore
from std_msgs.msg import Float64

# Each joint's command topic
TOPICS = {
    "Joint_1": "/Joint_1/command",
    "Joint_2": "/Joint_2/command",
    "Joint_3": "/Joint_3/command",
    "Joint_4": "/Joint_4/command",
    "Joint_5": "/Joint_5/command",
}

if __name__ == "__main__":
    rospy.init_node("joint_publisher_fixed", anonymous=True)

    # Create one publisher per joint
    pubs = {j: rospy.Publisher(t, Float64, queue_size=10) for j, t in TOPICS.items()}
    rate = rospy.Rate(10)  # Hz

    # Define fixed joint angles (in radians)
    joint_angles = {
        "Joint_1": -math.pi / 6,   # 30 degrees
        "Joint_2": -math.pi / 4,   # 45 degrees
        "Joint_3": -math.pi / 3,   # 60 degrees
        "Joint_4": 0.0,
        "Joint_5": 0.0,
    }

    rospy.loginfo("Joint publisher started — fixed angles (rad): %s",
                  {k: round(v, 4) for k, v in joint_angles.items()})

    rospy.sleep(0.5)  # allow publishers to connect

    while not rospy.is_shutdown():
        for j, pub in pubs.items():
            pub.publish(joint_angles[j])
        rate.sleep()

