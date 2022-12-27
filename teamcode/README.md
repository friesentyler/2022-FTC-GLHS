# There's not much to this code

Webcam example implements the image processing and allows it to control the motors to the robot.

Sample pipeline processes the image using an implementation of OpenCV specifically designed for the REV robots. The image processing filters for anything
yellow and then greyscales the image. The remaining white objects in the image are detected and have their pixel coordinates returned.

Before starting this project I thought I should use AI object detection, but I realized it would be much simpler to implement the image processing without
the AI.
