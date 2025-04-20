Q-1
Here we are using an OAK camera which consists of DEPTH AI API which processes the captured video using OPENCV so it inistializes the Pipline by first it takes care of the left and right camera and from their images i havent taken care of the parallax error but then the image is created and the image which has an RGB reso of 640*480 is converted to BGR format which is Black and white format so each frame is converted to black and white with the help of kernels which is then done gaussian blur and for depth finding is used canny edge detection to highlight edges and find depth in image.

Q-2
SO here again similar to solving process of  the earlier questions the only thing which has changed is here we just have to publish and subscribe the images jsut like done in the previous task.
Here i have used cv2waitkey(1) such that data which can be taken form publisher with an interval of 1 millisec such that computer doesnt crash and also i am using CV2 bridge to convert everything from ROS to OpenCV. cv2.destroyAllWindows()` closes the OpenCV tabs when the script is finished.


