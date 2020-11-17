#!/usr/bin/env python3

import roslib
import sys
import rospy
import cv2
import numpy as np
from image1 import *
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, Float64, Int64MultiArray
from cv_bridge import CvBridge, CvBridgeError


class image_converter:

  # Defines publisher and subscriber
  def __init__(self):
    # initialize the node named image_processing
    rospy.init_node('image_processing', anonymous=True)
    # initialize a publisher to send images from camera2 to a topic named image_topic2
    self.image_pub2 = rospy.Publisher("image_topic2",Image, queue_size = 1)
    # initialize a subscriber to recieve messages rom a topic named /robot/camera1/image_raw and use callback function to recieve data
    self.image_sub2 = rospy.Subscriber("/camera2/robot/image_raw",Image,self.callback2)
    # initialize the bridge between openCV and ROS
    self.bridge = CvBridge()

    # initialize publishers to send blob centers to image1
    self.yellow_blob_pub = rospy.Publisher("/camera2/yellow_blob_data", Int64MultiArray, queue_size = 1)
    #self.blue_blob_pub = rospy.Publisher("/camera2/blue_blob_data", list, queue_size = 1)
    #self.green_blob_pub = rospy.Publisher("/camera2/green_blob_data", list, queue_size = 1)
    #self.red_blob_pub = rospy.Publisher("/camera2/red_blob_data", list, queue_size = 1)
    #self.target_blob_pub = rospy.Publisher("/camera2/target_blob_data", list, queue_size = 1)
    #self.obstacle_blob_pub = rospy.Publisher("/camera2/obstacle_blob_data", list, queue_size = 1)

    


  # Recieve data, process it, and publish
  def callback2(self,data):
    # Recieve the image
    try:
      self.cv_image2 = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    # Uncomment if you want to save the image
    #cv2.imwrite('image_copy.png', cv_image)
    im2=cv2.imshow('window2', self.cv_image2)
    cv2.waitKey(1)

    # Publish blob data
    self.yellow_blob = Int64MultiArray()
    self.yellow_blob.data = detect_yellow_center(self.cv_image2)
    self.yellow_blob_pub.publish(self.yellow_blob)

    # Publish the results
    try: 
      self.image_pub2.publish(self.bridge.cv2_to_imgmsg(self.cv_image2, "bgr8"))
    except CvBridgeError as e:
      print(e)

    

    

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


