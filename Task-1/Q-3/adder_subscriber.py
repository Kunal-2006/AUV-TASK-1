# adder_subscriber.py
import rospy
from std_msgs.msg import Int32

def callback(data):
    result = data.data + 5
    rospy.loginfo(f"Received {data.data}, added 5: {result}")

def subscriber():
    rospy.init_node('adder_subscriber', anonymous=True)
    rospy.Subscriber('multiplied_by_10', Int32, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        subscriber()
    except rospy.ROSInterruptException:
        pass
