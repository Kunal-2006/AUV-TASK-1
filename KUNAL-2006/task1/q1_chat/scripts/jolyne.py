#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo("Joestar: %s", data.data)

def talker():
    pub = rospy.Publisher('chat_topic', String, queue_size=10)
    rospy.init_node('jolyne_node', anonymous=True)
    rospy.Subscriber('chat_topic', String, callback)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        msg = input("Jolyne: ")
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
