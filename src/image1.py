#!/usr/bin/env python3

import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, Float64
from cv_bridge import CvBridge, CvBridgeError


class image_converter:

  # Defines publisher and subscriber
  def __init__(self):
    # initialize the node named image_processing
    rospy.init_node('image_processing', anonymous=True)
    # initialize a publisher to send images from camera1 to a topic named image_topic1
    self.image_pub1 = rospy.Publisher("image_topic1",Image, queue_size = 1)
    # initialize a subscriber to recieve messages rom a topic named /robot/camera1/image_raw and use callback function to recieve data
    self.image_sub1 = rospy.Subscriber("/camera1/robot/image_raw",Image,self.callback1)
    # initialize the bridge between openCV and ROS
    self.bridge = CvBridge()
    # initialize publishers to send inputs to the robot's joints
    self.joint2_pub = rospy.Publisher("/robot/joint2_position_controller/command", Float64, queue_size=10)
    self.joint3_pub = rospy.Publisher("/robot/joint3_position_controller/command", Float64, queue_size=10)
    self.joint4_pub = rospy.Publisher("/robot/joint4_position_controller/command", Float64, queue_size=10)

    self.start_time = rospy.get_time()

  # Recieve data from camera 1, process it, and publish
  def callback1(self,data):
    # Recieve the image
    try:
      self.cv_image1 = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    
    # Uncomment if you want to save the image
    #cv2.imwrite('image_copy.png', cv_image)

    # Used to test rgb_normalization function
    rgb_normalize(self.cv_image1)

    c = detect_orange_center(self.cv_image1)
    self.cv_image1[c[1] - 1:c[1] + 1, c[0] - 1: c[0] + 1, 0] = 0
    self.cv_image1[c[1] - 1:c[1] + 1, c[0] - 1: c[0] + 1, 1] = 0
    self.cv_image1[c[1] - 1:c[1] + 1, c[0] - 1: c[0] + 1, 2] = 255

    im1=cv2.imshow('window1', self.cv_image1)
    cv2.waitKey(1)
    # Publish the results
    try: 
      self.image_pub1.publish(self.bridge.cv2_to_imgmsg(self.cv_image1, "bgr8"))
    except CvBridgeError as e:
      print(e)

    # Publish the robot's joint inputs
    self.joint2 = Float64()
    self.joint3 = Float64()
    self.joint4 = Float64()

    current_time = rospy.get_time() - self.start_time

    self.joint2.data = (np.pi / 2) * np.sin((np.pi / 15) * current_time)
    self.joint3.data = (np.pi / 2) * np.sin((np.pi / 18) * current_time)
    self.joint4.data = (np.pi / 2) * np.sin((np.pi / 20) * current_time)

    self.joint2_pub.publish(self.joint2)
    self.joint3_pub.publish(self.joint3)
    self.joint4_pub.publish(self.joint4)

def rgb_normalize(image):
  # Use RGB normalization to handle the varying lighting
  rgb_norm_image = np.zeros(np.shape(image),np.float64)

  b,g,r = cv2.split(image)

  rgb_norm_image[:,:,0] = np.divide(b, b + g + 1.2*r + np.ones(np.shape(r))) * 255 # Add one to each element to deal with division by 0
  rgb_norm_image[:,:,1] = np.divide(g, b + g + 1.2*r + np.ones(np.shape(r))) * 255
  rgb_norm_image[:,:,2] = np.divide(r, b + g + 1.2*r + np.ones(np.shape(r))) * 255

  rgb_norm_image = cv2.convertScaleAbs(rgb_norm_image)

  # imshow used for testing purposes:
  cv2.imshow('normalization', rgb_norm_image)
  cv2.waitKey(1)

  return rgb_norm_image

def detect_yellow_center(image):
  # Color boundaries
  yellow_lower = np.array([0,75,75])
  yellow_upper = np.array([30,255,255])
  # Thresholding and removing noise
  thresholded = cv2.inRange(image, yellow_lower, yellow_upper)
  thresholded = cv2.erode(thresholded, np.ones(3, np.uint8))
  thresholded = cv2.dilate(thresholded, np.ones(3, np.uint8))
  # Finding the center point
  moments = cv2.moments(thresholded)
  cx = int(moments['m10']/moments['m00'])
  cy = int(moments['m01']/moments['m00'])
  return np.array([cx,cy])

def detect_blue_center(image):
  # Color boundaries
  blue_lower = np.array([75,0,0])
  blue_upper = np.array([255,75,75])
  # Thresholding and removing noise
  thresholded = cv2.inRange(image, blue_lower, blue_upper)
  thresholded = cv2.erode(thresholded, np.ones(3, np.uint8))
  thresholded = cv2.dilate(thresholded, np.ones(3, np.uint8))
  # Finding the center point
  moments = cv2.moments(thresholded)
  cx = int(moments['m10']/moments['m00'])
  cy = int(moments['m01']/moments['m00'])
  return np.array([cx,cy])
    
def detect_green_center(image):
  # Color boundaries
  green_lower = np.array([0,75,0])
  green_upper = np.array([75,255,75])
  # Thresholding and removing noise
  thresholded = cv2.inRange(image, green_lower, green_upper)
  thresholded = cv2.erode(thresholded, np.ones(3, np.uint8))
  thresholded = cv2.dilate(thresholded, np.ones(3, np.uint8))
  # Finding the center point
  moments = cv2.moments(thresholded)
  cx = int(moments['m10']/moments['m00'])
  cy = int(moments['m01']/moments['m00'])
  return np.array([cx,cy])

def detect_red_center(image):
  # Color boundaries
  red_lower = np.array([0,0,100])
  red_upper = np.array([50,50,255])
  # Thresholding and removing noise
  thresholded = cv2.inRange(image, red_lower, red_upper)
  thresholded = cv2.erode(thresholded, np.ones(3, np.uint8))
  thresholded = cv2.dilate(thresholded, np.ones(3, np.uint8))
  # Finding the center point
  moments = cv2.moments(thresholded)
  cx = int(moments['m10']/moments['m00'])
  cy = int(moments['m01']/moments['m00'])
  return np.array([cx,cy])

def detect_orange_center(image):
  # Color boundaries
  orange_lower = np.array([0,50,50])
  orange_upper = np.array([50,80,255])
  # Thresholding and removing noise
  thresholded = cv2.inRange(image, orange_lower, orange_upper)
  thresholded = cv2.erode(thresholded, np.ones(3, np.uint8))
  thresholded = cv2.dilate(thresholded, np.ones(3, np.uint8))
  
  ones = np.argwhere(thresholded)
  y_max = np.max(ones[:,1])
  y_min = np.min(ones[:,1])
  x_max = np.max(ones[:,0])
  x_min = np.min(ones[:,0])

  cv2.imshow("all", thresholded)

  thresholded = thresholded[x_min - 10 : x_max + 10, y_min - 10: y_max + 10]

  cv2.imshow("target", thresholded)
  cv2.waitKey(1)

  # Finding the center point
  moments = cv2.moments(thresholded)
  cx = int(moments['m10']/moments['m00'])
  cy = int(moments['m01']/moments['m00'])
  return np.array([cx,cy])


# call the class
def main(args):
  ic = image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

# run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)

