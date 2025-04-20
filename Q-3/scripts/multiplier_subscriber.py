#!/usr/bin/env python3
# multiplier_subscriber.py
import rospy
from std_msgs.msg import Int32

def callback(data):
    result = data.data * 10
    rospy.loginfo(f"Received {data.data}, multiplied by 10: {result}")
    pub.publish(result)

def subscriber():
    rospy.init_node('multiplier_subscriber', anonymous=True)
    rospy.Subscriber('multiples_of_2', Int32, callback)
    global pub
    pub = rospy.Publisher('multiplied_by_10', Int32, queue_size=10)
    rospy.spin()

if __name__ == '__main__':
    try:
        subscriber()
    except rospy.ROSInterruptException:
        pass
