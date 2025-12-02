 #!/usr/bin/env python3
import rospy, math
from std_msgs.msg import Float64

# Joint command topics you already confirmed:
TOPICS = [
    "/Joint_1/command",
    "/Joint_2/command",
    "/Joint_3/command",
    "/Joint_4/command",
    "/Joint_5/command",
]

if __name__ == "__main__":
    rospy.init_node("joint_publisher_fixed_30deg", anonymous=True)
    pubs = [rospy.Publisher(t, Float64, queue_size=10) for t in TOPICS]

    # 30 degrees in radians (controllers expect radians)
    target = math.radians(30.0)

    # Give publishers time to connect
    rospy.sleep(0.5)
    rospy.loginfo("Publishing constant 30° (%.4f rad) to all joints", target)

    rate = rospy.Rate(10)  # Hz
    while not rospy.is_shutdown():
        for p in pubs:
            p.publish(target)
        rate.sleep()

