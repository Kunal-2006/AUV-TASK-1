#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int32

def callback(data):
    result = data.data + 5
    rospy.loginfo(f"Final Output: {result}")

def listener():
    rospy.init_node('final_subscriber', anonymous=True)
    rospy.Subscriber('/multiplied_topic', Int32, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
