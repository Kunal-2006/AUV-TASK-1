#q-1
Here we are using an OAK camera which consists of DEPTH AI API which processes the captured video using OPENCV so it inistializes the Pipline by first it takes care of the left and right camera and from their images i havent taken care of the parallax error but then the image is created and the image which has an RGB reso of 640*480 is converted to BGR format which is Black and white format so each frame is converted to black and white with the help of kernels which is then done gaussian blur and for depth finding is used canny edge detection to highlight edges and find depth in image.

#q-2

