import cv2  # OpenCV for image processing and display
import depthai as dai  # DepthAI SDK to interface with Luxonis cameras

# 1. Create pipeline
pipeline = dai.Pipeline()  # This sets up the data flow for the Luxonis camera

# 2. Define camera node (Color camera)
cam_rgb = pipeline.createColorCamera()  # This node captures RGB video
cam_rgb.setPreviewSize(640, 480)        # Set output resolution
cam_rgb.setInterleaved(False)           # Ensures output format is non-interleaved (standard)
cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)  # Output in BGR format for OpenCV

# 3. Create XLinkOut node to send frames to host
xout = pipeline.createXLinkOut()        # Output stream node to send data to the host machine
xout.setStreamName("video")             # Name the stream for access later
cam_rgb.preview.link(xout.input)        # Connect the camera's preview output to the stream

# 4. Start the pipeline
with dai.Device(pipeline) as device:  # Opens the connection to the physical camera
    video_queue = device.getOutputQueue(name="video", maxSize=4, blocking=False)
    
    print("Luxonis camera started. Press 'q' to quit.")

    while True:
        frame = video_queue.get().getCvFrame()  # Get frame as OpenCV image from Luxonis stream

        # Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Apply Gaussian blur
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)

        # Apply Canny edge detection
        edges = cv2.Canny(blurred, 100, 200)

        # Display original and edge-detected images
        cv2.imshow("Luxonis RGB", frame)
        cv2.imshow("Edges", edges)

        # Exit on 'q' key press
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

# 5. Clean up windows after breaking the loop
cv2.destroyAllWindows()
