#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int32

def publisher():
    pub = rospy.Publisher('/number_topic', Int32, queue_size=10)
    rospy.init_node('multiplier_publisher', anonymous=True)
    rate = rospy.Rate(1)  # 1 Hz
    num = 1

    while not rospy.is_shutdown():
        value = num * 2
        rospy.loginfo(f"Publisher: {value}")
        pub.publish(value)
        num += 1
        rate.sleep()

if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
