#!/usr/bin/env python


##############################################################################################################################################
#	circles_trendline_trackbars_final.py												                                                     #
#																	                                                                         #
#	This program detects a test pattern (six circles on a plane) on an image with the hough circle transformation from Open CV.	             #
#	The images were published by a ROS node from a basler camera based on the pylon-ros-camera driver.				                         # 
#	Because the camera system is running in a ROS environment it is necessary to create a cv_bridge. 				                         #
#	The cv_bridge converts the ROS-image format temporally into a Open CV-image format, which could handle several Open CV algorithms.       #
#	With the the informations of the detected circles a rectangle could be computed around the 6 detected circles of the test pattern.       # 
#	Also the distance between the test pattern and the camera could be computed with these imformations. 				                     #  	
#															         	                                                                     #
#	created by: Wilhelm Geugis, Hochschule Karlsruhe, Studiengang:FZTB 					                                    			     #
#	E-Mail: gewi1011@hs-karlsruhe.de				         							                                                     #
#	Date: 14.01.2021														                                                                 #
##############################################################################################################################################


from __future__ import print_function
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from std_msgs.msg import Int8, Int16
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
    
class image_bridge:

  def __init__(self):
    
    # creating Publisher with Topic "image_circles_opencv" for each message to publish 
    # queue_size to avoid a bottle neck and loss of publishing images -> buffer
    self.image_pub = rospy.Publisher("image_bridge_opencv_02/detected_circles_img",Image, queue_size=2) 
    self.flag_pub = rospy.Publisher("image_bridge_opencv_02/detected_circles_flag", Int8, queue_size=2)
    self.distance_pub = rospy.Publisher("image_bridge_opencv_02/distance",Int16, queue_size=2)
    

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera_02/pylon_camera_node/image_raw",Image,self.callback)
    
    
  
  def callback(self,data):
       
    flag=0
    distance=0
    
    try:
      
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

    except CvBridgeError as e:
      print(e)


      
    #------------------------------------------------------------------------------------------------------------      
    # ####################                   OpenCV Code                 ########################################
    #------------------------------------------------------------------------------------------------------------
    
    #--------convert image into grayscale image-------------------------------------------------------------------
    gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

    #--------median blur filter-----------------------------------------------------------------------------------
    
    #apply a median blur filter to reduce the noise and avoid false cirlce detection, makes image smoother
   
    #image_medianBlur =	cv.medianBlur(src, ksize)
    
    #src: input image
    #image_medianBlur: output image 
    #ksize: size of the filter/aperture, ksize x ksize, must be greater than 1, e.g. 3,5,7
    
    gray = cv2.medianBlur(gray, 5)

    

    #--------Hough Circle Transformation--------------------------------------------------------------------------
    
    #detailed information for the parameters of the following method:

    #cv2.HoughCircles(image, method, dp, minDist, param1, param2, minRadius, maxRadius)

    #image: grayscale 8-bit image
    #method: method to detect circles, only implemented method is actually cv2.HOUGH_GRADIENT
    #dp: parameter of the cv2.HOUGH_GRADIENT method
    #minDist: minimum distance between the center (x, y) coordinates of detected circles
    #param1: threshold value for the cv2.HOUGH_GRADIENT method
    #param2: gradient value for the cv2.HOUGH_GRADIENT method
    #minRadus: minimum size of radius [pixel]
    #maxRadius: maximum size of radius [pixel]

    #values coming from the trackbar positions
    p1=cv2.getTrackbarPos("param1", "detected_circles_02")
    p2=cv2.getTrackbarPos("param2", "detected_circles_02")
     
    circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, 16, param1=p1, param2=p2, minRadius=1, maxRadius=200)
    
    
    #--------Draw the detected circles, centerpoints and a green rectangle when the test pattern has been found-----

    # The dimensions and the origin of the rectangle are based on the distance between the circles. 
    # The distances between the outline of each circle on the test pattern between each other outline of the circles are the same.
    # So the rectangle has the same distance to the outlines of the circles. 
    # To compute this we used to calculate the average distance between the outline of the six circles
    # The distance between the camera and the test pattern, is calculated based on the radius of the detected circles on the image. 
    # Therefore the average radius was also calculated to compensate for small deviations in the radius.  

    if circles is not None:
        rospy.loginfo('detected circles: ' + str(len(circles[0])))
        circles = np.uint16(np.around(circles)) #creating numpy array with data of hough circle transformation
                
        #init values
        min_x=2000
        max_x=0
        min_y=2000
        max_y=0
        total_radius=0 #variable to sum up all detected radius, later we can compute with this value the average radius 
        total_dst_centerpt=0 #variable to sum up all distances between all centers of the detected circles, later we can compute with this value the average radius

        for i in circles[0, :]:
            center = (i[0], i[1])
                        
            # drawing circle center
            cv2.circle(cv_image, center, 1, (0, 100, 100), 2) #params: cv2.circle(image, center_coordinates, radius, color, thickness)

            # drawing circle outline
            radius = i[2]
            cv2.circle(cv_image, center, radius, (255, 0, 255), 2)

            #Values to calculate the green rectangle
            total_radius=total_radius + i[2]
            center_x=i[0]
            center_y=i[1]
            
            #calcualte minimum/maximum of x/y coordinates centerpoints
            if center_x < min_x:
              min_x=center_x
            if center_x > max_x:
              max_x=center_x

            if center_y < min_y:
              min_y=center_y
            if center_y > max_y:
              max_y=center_y  
            
        #draw green rectangular around the detected circles, when 6 cricles are detected:

        if len(circles [0])==6:
            
	    flag=1 #flag that 6 circles were detected
            #rospy.loginfo('detection_flag: '+str(self.flag)) 
         
            #search for the smallest distance between the coordinates of the centers with a tolerance of 10px
            #grab one coordinate of a center:[x-coordiante, y-coordinate]:
            #x-coordinate: circles[0][:][0] y-coordinate: circles[0][:][1]
            
	    #temporary variables for x and y coordinates
	    tempx=circles[0][1][0]
            tempy=circles[0][1][1]

            #init: random high values for min/max calculation
            dist_min_x=3000
            dist_min_y=3000
            
            for k in range(5,0,-1): #evaluate every centerpoint coordinate, if it is the smallest
              center_xk=circles[0][k][0]
              center_yk=circles[0][k][1]
              
              if (center_xk > tempx+10) or (center_xk < tempx-10):
                dist_x=abs(center_xk-tempx)
                
              
                if dist_x < dist_min_x and dist_x!=0:
                  dist_min_x=dist_x

              if (center_yk > tempy+10) or (center_yk < tempy-10):
                dist_y=abs(center_yk-tempy)
                
              
                if dist_y < dist_min_y and dist_y!=0:
                  dist_min_y=dist_y
            
            #----------------- calculate average radius of the circles-----------------------------------------
            avrg_radius=total_radius/6 

            avrg_dst_centerpt=(dist_min_y+dist_min_x)/2 #average distance between centerpoints of the circles
            avrg_dist_circles=avrg_dst_centerpt-avrg_radius #calculating average distance between the edge of one circle to the next one

            #only top left rectangle point and bottom right point necessary to compute the rectangle
            left_top_rect_pt=(min_x-avrg_dist_circles,max_y+avrg_dist_circles)
            right_bottom_rect_pt=(max_x+avrg_dist_circles,min_y-avrg_dist_circles)
            
            #----------------output loginfo for information in terminal----------------------------- 
            #rospy.loginfo('averg_dist_centerpnt :'+str(avrg_dst_centerpt))
            #rospy.loginfo('left top: '+str(left_top_rect_pt)) #left top origin of the rectangle
            #rospy.loginfo('right bottom: '+str(right_bottom_rect_pt)) #bottom right origin of the rectangle
            rospy.loginfo('avrg_radius: '+str(avrg_radius))
            #rospy.loginfo('avrg_dist_circles: '+str(avrg_dist_circles))

            #drawing green rectangle with a thickness of 3 with the coordinate points of the top left- and the bottom rigtht corner
            cv2.rectangle(cv_image,left_top_rect_pt,right_bottom_rect_pt,(0,255,0),3)
            
	    #----------------calculation of the distnace to the testpattern with a trendline function-----------
            
            #uncomment the following line if you are creating a look up table for the trendline function, then the average radius is displayed and its easier to note these values
	    #cv2.putText(cv_image, str(avrg_radius), (200,500), font, 11, color, 6)
            
            #output distance with the knowledge of a look-up-table of the average radius of the circles
            #values of the look-up-table are formulated with an "trendline potency" in EXCEL: distance=1030,3*averag_radius^(-0,875)
            distance=round(2340.3*avrg_radius**-0.993)
            cv2.putText(cv_image, 'distance to camera: '+str(distance)+'cm', (50,200), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 3)
	    #rospy.loginfo('distance to camera :'+str(self.distance))

        else:
	    flag=0

        #Display the number of the detected circles at the top left corner in green
        #cv2.putText(image, text, org, font, fontScale, color, thickness, lineType, bottomLeftOrigin)          
        cv2.putText(cv_image, '# of detected circles: '+str(len(circles[0])), (50,100), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 3)
         
    #--------Display detected circles and text on image-----------------------------------------------------
    cv2.imshow("detected_circles_02", cv_image)
    
    
    cv2.waitKey(3)
    
    #-----------------------------------------------------------------------------------------------------------
    # End of Open CV code    
    #-----------------------------------------------------------------------------------------------------------


    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
      self.flag_pub.publish(flag)
      self.distance_pub.publish(distance)
      
    except CvBridgeError as e:
      print(e)

def empty(a): #empty method to actualize the trackbars
  pass
    

def main(args):

  
  #creating a window
  cv2.namedWindow("detected_circles_02",cv2.WINDOW_NORMAL)
  cv2.resizeWindow("detected_circles_02",800,800)
  

  # creating trackbars to control the value of the parameters of the hough circle transformation
  # the user can adapt the value manually, especially when there are different light conditions
  # cv2.createTrackbar("name of trackbar","name of window",initial_value,max_value,trigger_function)
  cv2.createTrackbar("param1","detected_circles_02",220,255,empty)  
  cv2.createTrackbar("param2","detected_circles_02",39,100,empty)	

  rospy.init_node('image_bridge_02', anonymous=True)
  ib = image_bridge() #creating object of the class "image_bridge"
  rospy.loginfo('image_bridge_02 node started')
  
  try:
    rospy.spin() #run node endless until we shutt down the node with STRG+C
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
