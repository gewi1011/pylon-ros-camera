# Commissioning of driver assistance cameras and implementation of object detection under ROS and OpenCV

As part of my bachelor thesis on camera-based driver assistance systems, two mono cameras have been attached to a measurement vehicle. The two individual mono cameras (Basler daA1600-60uc (USB 3.0)) should read out with the pylon-ros-camera driver in the open source environment ROS and their data visualised. The individual frames of the cameras can then be further processed with the help of OpenCV. In this project the cameras should recognise a pattern on a test sign and determine from this the distance of the test sign to the vehicle.

## necessary files and programs to run the system:

The whole system is running with Ubuntu 18.04, ROS Melodic 1.14.9, Open CV 3.2.0, Nvidia Driver 450.80.02, USB-Interface: Fresco FL1100

after installing the pylon-ros-camera driver like in the README of the branch "devel",
you have to install the two ROS packages: 

-cv_bridge (http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython)

-camera_calibration (http://wiki.ros.org/camera_calibration)


A separate package that can be used to start image processing scripts can be created, for example, as for the "camera_vision_opencv" package: 

`sudo apt-get install ros-kinetic-catkin python-catkin-tools`
`cd ~/catkin_ws/src` 
`catkin create pkg camera_vision_opencv --catkin-deps rospy cv_bridge` 
`cd camera_vision_opencv`
`mkdir nodes`

In the folder "nodes" under ~/catkin_ws/src/camera_vision_opencv/ you can now store the Python scripts from this branch (-> camera_vision_opencv/nodes).
It is important that the Python files are made executable!


then you could add the following files to the pylon-ros-camera directory and the image processing package (camera_vision_opencv): 

-**config files**:
add the two config files to the folder of the directoy: ~/catkin_ws/src/pylon-ros-camera-devel/pylon_camera/config
attention: 

-**launch file**:
add the launch file to the folder ~/catkin_ws/src/pylon-ros-camera-devel/pylon_camera/launch

-**calibration-files**: after creating with the "camera_calibration" package your own calibration files, you have to add them to the following directory: ~/catkin_ws/src/pylon-ros-camera-devel/pylon_camera/config/calibration_files

Then you have to made some adjustments in the config file of each camera: in line 13 at the parameter "camera_info_url" should be your own directory of the calibration file (be careful with different ubuntu user names!) 


## to run the whole system execute the following steps (each in a own terminal):

1) **launch the two cameras**:
`roslaunch pylon_camera pylon_camera_node_multiple_cameras.launch`

2) **run the python scripts for image detection**:
`rosrun camera_vision_opencv test_object_detection_01.py`
`rosrun camera_vision_opencv test_object_detection_01.py`

3) **run the python gui of all cameras**:
`rosrun camera_vision_opencv GUI_all_cameras_01_02.py`


**This is only a general overview of this project,
if you have any question feel free to contact me!**
