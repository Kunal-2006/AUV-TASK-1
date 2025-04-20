# publisher.py
import rospy
from std_msgs.msg import Int32

def publisher():
    rospy.init_node('publisher_node', anonymous=True)
    pub = rospy.Publisher('multiples_of_2', Int32, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz rate
    num = 2

    while not rospy.is_shutdown():
        rospy.loginfo(f"Publishing: {num}")
        pub.publish(num)
        num += 2  # Increment to the next multiple of 2
        rate.sleep()

if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
