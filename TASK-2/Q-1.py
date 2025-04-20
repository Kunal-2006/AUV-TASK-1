import cv2

# Initialize camera capture object
cap = cv2.VideoCapture(0)  # 0 for default camera

# Verify camera connection
if not cap.isOpened():
    print("Error: Camera not accessible")
    exit()

# Continuous frame capture loop
while True:
    ret, frame = cap.read()
    if not ret:
        print("Frame capture failed")
        break
    
    # Display captured frame
    cv2.imshow('Live Feed', frame)
    
    # Exit on 'q' keypress
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release resources
cap.release()
cv2.destroyAllWindows()
while True:
    ret, frame = cap.read()
    if not ret:
        break
    
    # Convert to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Apply Gaussian blur
    blurred = cv2.GaussianBlur(gray, (5,5), 0)
    
    # Canny edge detection
    edges = cv2.Canny(blurred, 100, 200)
    
    # Display results
    cv2.imshow('Original', frame)
    cv2.imshow('Edges', edges)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
