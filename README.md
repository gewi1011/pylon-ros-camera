# Commissioning of driver assistance cameras and implementation of object detection under ROS and OpenCV

As part of my bachelor thesis on camera-based driver assistance systems, mono cameras have been attached to a measurement vehicle. The two individual mono cameras (Basler daA1600-60uc (USB 3.0)) should read out with the pylon-ros-camer driver in the open source environment ROS and their data visualised. The individual frames of the cameras can then be further processed with the help of OpenCV. In this project the cameras should recognise a pattern on a test sign and determine from this the distance of the test sign to the vehicle.

#In the following all necessary files and programs are listed:
the goal of this project is to 

after installing the pylon-ros-camera driver like in the README of the branch "devel",
you could add the following files:

-config files:
add the config files to the folder 

-launch file:
add the launch file to the folder

-calibration-files:

run the python scripts:

A separate package that can be used to start image processing scripts can be created, for example, as for the "camera_vision_opencv" package: sudo apt-get install ros-kinetic-catkin python-catkin-tools cd ~/catkin_ws/src catkin create pkg camera_vision_opencv --catkin-deps rospy cv_bridge cd camera_vision_opencv mkdir nodes
In the folder "nodes" under ~/catkin_ws/src/camera_vision_opencv/ you can now store the Python scripts, such as the test script "image_bridge_cv_circle_test.py".
It is important that the Python file is made executable. This is possible either via: Right click on the file > Properties > Permissions > Execute > "Allow executing file as program" or as a command in the terminal:
chmod u+x ~/catkin_ws/src/camera_vision_opencv/nodes/image_bridge_cv_c ircle_test.py

rosrun camera_vision_opencv image_bridge_cv_circle_test.py
