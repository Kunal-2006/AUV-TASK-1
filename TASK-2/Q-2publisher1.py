#!/usr/bin/env python3
import rospy
import cv2
import depthai as dai
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def publish_edge_image():
    # 1. Create pipeline
    pipeline = dai.Pipeline()  # Create a DepthAI pipeline to configure camera data flow

    # 2. Define camera node (Color camera)
    cam_rgb = pipeline.createColorCamera()  # Create RGB camera node
    cam_rgb.setPreviewSize(640, 480)        # Set output resolution
    cam_rgb.setInterleaved(False)           # Use non-interleaved format (standard BGR)
    cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)  # Set output color format to BGR

    # 3. Create XLinkOut node to send frames to the host machine
    xout = pipeline.createXLinkOut()        # Create XLinkOut node
    xout.setStreamName("video")             # Name the stream for easy reference
    cam_rgb.preview.link(xout.input)        # Link the camera preview to the XLinkOut stream

    # 4. Start the DepthAI device and create a video queue for processing frames
    with dai.Device(pipeline) as device:
        video_queue = device.getOutputQueue(name="video", maxSize=4, blocking=False)
        
        # Initialize ROS node
        rospy.init_node('edge_publisher', anonymous=True)
        
        # Initialize ROS Publisher (publishing to /edge_feed)
        pub = rospy.Publisher('edge_feed', Image, queue_size=10)

        # Create CvBridge for ROS-to-OpenCV conversions
        bridge = CvBridge()

        print("Luxonis camera started. Publishing edge-detected images to 'edge_feed'...")

        while not rospy.is_shutdown():
            # 5. Get frame from the camera stream
            frame = video_queue.get().getCvFrame()  # Convert DepthAI frame to OpenCV image

            # 6. Convert frame to grayscale (for edge detection)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # 7. Apply Gaussian blur to reduce noise before edge detection
            blurred = cv2.GaussianBlur(gray, (5, 5), 0)

            # 8. Apply Canny edge detection
            edges = cv2.Canny(blurred, 100, 200)

            # 9. Convert the processed edge image to a ROS Image message
            edge_msg = bridge.cv2_to_imgmsg(edges, encoding="mono8")  # Grayscale image (mono8)

            # 10. Publish the edge-detected image to the ROS topic
            pub.publish(edge_msg)

            # Display the frame and the edges in OpenCV windows
            cv2.imshow("RGB Frame", frame)
            cv2.imshow("Edges", edges)
            
            # Exit on 'q' key press
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    # Clean up when the loop ends
    cv2.destroyAllWindows()

if __name__ == '__main__':
    publish_edge_image()
