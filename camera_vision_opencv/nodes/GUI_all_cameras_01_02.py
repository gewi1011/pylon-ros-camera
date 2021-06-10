#!/usr/bin/env python

##############################################################################################################################################
#	GUI_all_camera_final.py														     #
#																	     #
#	creates a GUI of the topview of a car, an other ROS node detects a test pattern and the distance between the pattern and the camera.  #
#	Depending on the distance and the camera were the test pattern was detected, a a colored box is shown at this camera on the GUI      #
#																	     #
#	created by: Wilhelm Geugis, Hochschule Karlsruhe, Studiengang:FZTB 								     #
#	E-Mail: gewi1011@hs-karlsruhe.de				         							     #
#	Date: 14.01.2021														     #
##############################################################################################################################################



# coding: utf-8
from __future__ import print_function
import sys
import rospy
import cv2
import message_filters
from std_msgs.msg import Int8, Int16
    

def callback(flag01, distance01, flag02, distance02):

      flag01 = flag01.data #flag for camera_01
      distance01 = distance01.data #distance from camera_01 

      flag02 = flag02.data #flag for camera_02
      distance02 = distance02.data #distance from camera_02     

      image = cv2.imread('/home/gewi1011/catkin_ws/src/camera_vision_opencv/nodes/car_image_background_final.png')
      rospy.loginfo('distance: '+str(distance02))
      rospy.loginfo('object detected_flag:' + str(flag02))
      
      # if 6 circles were detected then the user get an information based on the distance and the flag, in which camera the test pattern was found.
      # depending on the color of the highlighted box around the cameras in the GUI, the test pattern is further or closer to the camera
      
      if flag01==1 and flag02==1: #pattern detected in both camera images-------
        
        #if 0<distance<70 the box will be red	
	if distance01 < 70 and distance01 > 0:
		cv2.rectangle(image, (530, 400), (620, 490), (0,0,255), 3)
        	cv2.putText(image, 'camera_01', (490, 380), cv2.FONT_HERSHEY_SIMPLEX, 1.1, (0,0,255), 3)
    	#if 70<distance<120 the box will be orange
	elif distance01 < 120 and distance01 > 70:
		cv2.rectangle(image, (530, 400), (620, 490), (0,127,255), 3)
        	cv2.putText(image, 'camera_01', (490, 380), cv2.FONT_HERSHEY_SIMPLEX, 1.1, (0,127,255), 3)
	#if 120<distance the box will be green
	elif distance01 > 120:
		cv2.rectangle(image, (530, 400), (620, 490), (0,255,0), 3)
        	cv2.putText(image, 'camera_01', (490, 380), cv2.FONT_HERSHEY_SIMPLEX, 1.1, (0,255,0), 3)
      
	#output of the distance:
	#cv2.putText(image, 'distance to object: '+str(distance01)+ 'cm', (400, 300), cv2.FONT_HERSHEY_SIMPLEX, 1.6, (0,0,0), 3) 

	#if 0<distance<70 the box will be red	
	if distance02 < 70 and distance02 > 0:
		cv2.rectangle(image, (830, 400), (920, 490), (0,0,255), 3)
        	cv2.putText(image, 'camera_02', (790, 380), cv2.FONT_HERSHEY_SIMPLEX, 1.1, (0,0,255), 3)
    	#if 70<distance<120 the box will be orange
	elif distance02 < 120 and distance02 > 70:
		cv2.rectangle(image, (830, 400), (920, 490), (0,127,255), 3)
        	cv2.putText(image, 'camera_02', (790, 380), cv2.FONT_HERSHEY_SIMPLEX, 1.1, (0,127,255), 3)
	#if 120<distance the box will be green
	elif distance02 > 120:
		cv2.rectangle(image, (830, 400), (920, 490), (0,255,0), 3)
        	cv2.putText(image, 'camera_02', (790, 380), cv2.FONT_HERSHEY_SIMPLEX, 1.1, (0,255,0), 3)
      
	#output of the distance:
	cv2.putText(image, 'distance to object: '+str(distance02)+ 'cm', (400, 300), cv2.FONT_HERSHEY_SIMPLEX, 1.6, (0,0,0), 3)
      

      elif flag01==1: #pattern was only detected in camera_01-------
	rospy.loginfo('object detected in camera_01')
	
	#if 0<distance<70 the box will be red	
	if distance01 < 70 and distance01 > 0:
		cv2.rectangle(image, (530, 400), (620, 490), (0,0,255), 3)
        	cv2.putText(image, 'camera_01', (490, 380), cv2.FONT_HERSHEY_SIMPLEX, 1.1, (0,0,255), 3)
    	#if 70<distance<120 the box will be orange
	elif distance01 < 120 and distance01 > 70:
		cv2.rectangle(image, (530, 400), (620, 490), (0,127,255), 3)
        	cv2.putText(image, 'camera_01', (490, 380), cv2.FONT_HERSHEY_SIMPLEX, 1.1, (0,127,255), 3)
	#if 120<distance the box will be green
	elif distance01 > 120:
		cv2.rectangle(image, (530, 400), (620, 490), (0,255,0), 3)
        	cv2.putText(image, 'camera_01', (490, 380), cv2.FONT_HERSHEY_SIMPLEX, 1.1, (0,255,0), 3)
      
	#output of the distance:
	cv2.putText(image, 'distance to object: '+str(distance01)+ 'cm', (400, 300), cv2.FONT_HERSHEY_SIMPLEX, 1.6, (0,0,0), 3)      


      elif flag02==1: #pattern was only detected in camera_02-------
	rospy.loginfo('object detected in camera_02')
	
	#if 0<distance<70 the box will be red	
	if distance02 < 70 and distance02 > 0:
		cv2.rectangle(image, (830, 400), (920, 490), (0,0,255), 3)
        	cv2.putText(image, 'camera_02', (790, 380), cv2.FONT_HERSHEY_SIMPLEX, 1.1, (0,0,255), 3)
    	#if 70<distance<120 the box will be orange
	elif distance02 < 120 and distance02 > 70:
		cv2.rectangle(image, (830, 400), (920, 490), (0,127,255), 3)
        	cv2.putText(image, 'camera_02', (790, 380), cv2.FONT_HERSHEY_SIMPLEX, 1.1, (0,127,255), 3)
	#if 120<distance the box will be green
	elif distance02 > 120:
		cv2.rectangle(image, (830, 400), (920, 490), (0,255,0), 3)
        	cv2.putText(image, 'camera_02', (790, 380), cv2.FONT_HERSHEY_SIMPLEX, 1.1, (0,255,0), 3)
      
	#output of the distance:
	cv2.putText(image, 'distance to object: '+str(distance02)+ 'cm', (400, 300), cv2.FONT_HERSHEY_SIMPLEX, 1.6, (0,0,0), 3)

      


      cv2.namedWindow('camera_overview',cv2.WINDOW_NORMAL)
      cv2.imshow('camera_overview',image)
      cv2.resizeWindow('camera_overview', 600,600)
    
      cv2.waitKey(3)  #waiting for 3 miliseconds
	
    
def main():
  
  
  rospy.init_node('camera_oveview', anonymous=True) #creating camera_overview node
  rospy.loginfo('camera_overview node started')
  
  #because this node subscribed to more than one topic, it is necessary to filter the messages and to synchronize them
  flag_sub_cam01 = message_filters.Subscriber('/image_bridge_opencv_01/detected_circles_flag', Int8)
  distance_sub_cam01 = message_filters.Subscriber('/image_bridge_opencv_01/distance', Int16)

  flag_sub_cam02 = message_filters.Subscriber('/image_bridge_opencv_02/detected_circles_flag', Int8)
  distance_sub_cam02 = message_filters.Subscriber('/image_bridge_opencv_02/distance', Int16)

  ts = message_filters.ApproximateTimeSynchronizer([flag_sub_cam01, distance_sub_cam01, flag_sub_cam02, distance_sub_cam02], 40, slop=0.01, allow_headerless=True)
  ts.registerCallback(callback)

  #run node continuously until it gets interrupted by a keyinput
  try:
    rospy.spin()    
  except KeyboardInterrupt:
    print("Shutting down")
  
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
