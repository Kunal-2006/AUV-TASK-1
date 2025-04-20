#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

def camera_publisher():
    rospy.init_node('edge_publisher', anonymous=True)
    pub = rospy.Publisher('edge_feed', Image, queue_size=10)
    bridge = CvBridge()
    
    cap = cv2.VideoCapture(0)
    rate = rospy.Rate(30)  # 30Hz
    
    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if ret:
            # Edge processing
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            edges = cv2.Canny(gray, 100, 200)
            
            # Convert to ROS message
            ros_image = bridge.cv2_to_imgmsg(edges, "mono8")
            pub.publish(ros_image)
            rate.sleep()

if __name__ == '__main__':
    try:
        camera_publisher()
    except rospy.ROSInterruptException:
        pass

