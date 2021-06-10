# Commissioning of driver assistance cameras and implementation of object detection under ROS and OpenCV

As part of my bachelor thesis on camera-based driver assistance systems, mono cameras have been attached to a measurement vehicle. The two individual mono cameras (Basler daA1600-60uc (USB 3.0)) should read out with the pylon-ros-camera driver in the open source environment ROS and their data visualised. The individual frames of the cameras can then be further processed with the help of OpenCV. In this project the cameras should recognise a pattern on a test sign and determine from this the distance of the test sign to the vehicle.

#In the following all necessary files and programs are listed:

The whole system is running with Ubuntu 18.04, ROS Melodic 1.14.9, Open CV 3.2.0, Nvidia Driver 450.80.02

after installing the pylon-ros-camera driver like in the README of the branch "devel",
you have to install the two ros packages: 
-cv_bridge (http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython)
-camera_calibration (http://wiki.ros.org/camera_calibration)

A separate package that can be used to start image processing scripts can be created, for example, as for the "camera_vision_opencv" package: 
sudo apt-get install ros-kinetic-catkin python-catkin-tools 
cd ~/catkin_ws/src catkin create pkg camera_vision_opencv --catkin-deps rospy cv_bridge 
cd camera_vision_opencv mkdir nodes
In the folder "nodes" under ~/catkin_ws/src/camera_vision_opencv/ you can now store the Python scripts from this repo (camera_vision_opencv/nodes).
It is important that the Python files are made executable!

then you could add the following files to the pylon-ros-camera and the 
-config files:
add the config files to the folder 

-launch file:
add the launch file to the folder

-calibration-files:

to run the whole system execute the following steps (each in a own terminal):

launch the cameras:
roslaunch pylon_camera pylon_camera_node_multiple_cameras.launch

run the python scripts for image detection:
rosrun camera_vision_opencv test_object_detection_01.py
rosrun camera_vision_opencv test_object_detection_01.py

run the python gui of all cameras:
rosrun camera_vision_opencv GUI_all_cameras_01_02.py
