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
    self.blob_center_publisher = rospy.Publisher("/camera2/blob_data", Int64MultiArray, queue_size = 1)

    


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
    self.blob_centers = Int64MultiArray()
    self.blob_centers.data = np.array([0 for _ in range(8)])
    self.blob_centers.data[0:2] = detect_yellow_center(self.cv_image2)
    self.blob_centers.data[2:4] = detect_blue_center(self.cv_image2)
    self.blob_centers.data[4:6] = detect_green_center(self.cv_image2)
    self.blob_centers.data[6:8] = detect_red_center(self.cv_image2)

    self.blob_center_publisher.publish(self.blob_centers)


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


