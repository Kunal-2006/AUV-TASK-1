#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

def image_callback(msg):
    bridge = CvBridge()
    try:
        # Convert ROS Image message to OpenCV grayscale image
        cv_image = bridge.imgmsg_to_cv2(msg, "mono8")
        
        # Display the image
        cv2.imshow("Edge Detection (Subscriber)", cv_image)
        cv2.waitKey(1)
    except Exception as e:
        print(f"[ERROR] Conversion failed: {e}")

def edge_subscriber():
    # Initialize ROS node
    rospy.init_node('luxonis_edge_subscriber', anonymous=True)

    # Subscribe to the 'edge_feed' topic
    rospy.Subscriber('edge_feed', Image, image_callback)

    # Keep the node running
    rospy.spin()

    # Clean up OpenCV windows when the node shuts down
    cv2.destroyAllWindows()

if __name__ == '__main__':
    edge_subscriber()
