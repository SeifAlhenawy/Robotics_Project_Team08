#!/usr/bin/env python3
import rospy, math # type: ignore
from std_msgs.msg import Float64

TOPICS = {
    "Joint_1": "/Joint_1/command",
    "Joint_2": "/Joint_2/command",
    "Joint_3": "/Joint_3/command",
    "Joint_4": "/Joint_4/command",
    "Joint_5": "/Joint_5/command",
}

if __name__ == "__main__":
    rospy.init_node("joint_publisher_param", anonymous=True)

    pubs = {j: rospy.Publisher(t, Float64, queue_size=10) for j, t in TOPICS.items()}
    rate = rospy.Rate(10)

    default_angles = {
        "Joint_1": 30.0,
        "Joint_2": 30.0,
        "Joint_3": 30.0,
        "Joint_4": 0.0,
        "Joint_5": 0.0,
    }

    for j, deg in default_angles.items():
        if not rospy.has_param(j):
            rospy.set_param(j, deg)

    rospy.loginfo(" Joint publisher started — change angles with 'rosparam set Joint_i <deg>'")

    while not rospy.is_shutdown():
        for j, pub in pubs.items():
            deg = rospy.get_param(j, default_angles[j])
            rad = math.radians(deg)
            pub.publish(rad)
        rate.sleep()



