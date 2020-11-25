#!/usr/bin/env python3

import roslib
import sys
import os
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, Float64, Int64MultiArray
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

    # initialize subscribers to recieve blob centers from image2
    self.blob_subscriber = rospy.Subscriber("/camera2/blob_data", Int64MultiArray, self.im2_update)
    self.blob_location = np.zeros((4,3))
    self.joint_angles = np.zeros(4)

    # initialize a publisher to publish the target's 3d location
    self.target_x_publisher = rospy.Publisher("/target/x_location", Float64, queue_size = 1)
    self.target_y_publisher = rospy.Publisher("/target/y_location", Float64, queue_size = 1)
    self.target_z_publisher = rospy.Publisher("/target/z_location", Float64, queue_size = 1)
    self.target_location = np.zeros(3)

    # meter value of a single unit
    self.distance_ratio = None


  # Recieve data from camera 1, process it, and publish
  def callback1(self,data):
    # Recieve the image
    try:
      self.cv_image1 = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    # Uncomment if you want to save the image
    #cv2.imwrite('image_copy.png', cv_image)


    # Visualisation of centers
    image = rgb_normalize(self.cv_image1)

    c = detect_target_center(image)
    #self.cv_image1[c[1] - 1:c[1] + 1, c[0] - 1: c[0] + 1, 0] = 0
    #self.cv_image1[c[1] - 1:c[1] + 1, c[0] - 1: c[0] + 1, 1] = 0
    #self.cv_image1[c[1] - 1:c[1] + 1, c[0] - 1: c[0] + 1, 2] = 255

    c = detect_yellow_center(image)
    #self.cv_image1[c[1] - 1:c[1] + 1, c[0] - 1: c[0] + 1, 0] = 0
    #self.cv_image1[c[1] - 1:c[1] + 1, c[0] - 1: c[0] + 1, 1] = 0
    #self.cv_image1[c[1] - 1:c[1] + 1, c[0] - 1: c[0] + 1, 2] = 255

    c = detect_blue_center(image)
    #self.cv_image1[c[1] - 1:c[1] + 1, c[0] - 1: c[0] + 1, 0] = 0
    #self.cv_image1[c[1] - 1:c[1] + 1, c[0] - 1: c[0] + 1, 1] = 0
    #self.cv_image1[c[1] - 1:c[1] + 1, c[0] - 1: c[0] + 1, 2] = 255

    c = detect_green_center(image)
    #self.cv_image1[c[1] - 1:c[1] + 1, c[0] - 1: c[0] + 1, 0] = 0
    #self.cv_image1[c[1] - 1:c[1] + 1, c[0] - 1: c[0] + 1, 1] = 0
    #self.cv_image1[c[1] - 1:c[1] + 1, c[0] - 1: c[0] + 1, 2] = 255

    c = detect_red_center(image)
    #self.cv_image1[c[1] - 1:c[1] + 1, c[0] - 1: c[0] + 1, 0] = 0
    #self.cv_image1[c[1] - 1:c[1] + 1, c[0] - 1: c[0] + 1, 1] = 255
    #self.cv_image1[c[1] - 1:c[1] + 1, c[0] - 1: c[0] + 1, 2] = 0


    cv2.imshow('window1', self.cv_image1)
    # cv2.imshow('window2', image)
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

    # self.joint2_pub.publish(self.joint2)
    # self.joint3_pub.publish(self.joint3)
    # self.joint4_pub.publish(self.joint4)

    # print(self.blob_location[0], self.blob_location[1])
    # print(self.blob_location[2] - np.array(
      # [3.5 * np.sin(self.joint2.data), 
      # -3.5 * np.sin(self.joint2.data) * np.cos(self.joint3.data),
      # 3.5 * np.cos(self.joint2.data) * np.cos(self.joint3.data)]))
    


  def im2_update(self,data):
    image1 = rgb_normalize(self.cv_image1)
    
    center_info_1 = np.array([detect_yellow_center(image1), detect_blue_center(image1), detect_green_center(image1), detect_red_center(image1)])
    center_info_2 = np.reshape(np.array(data.data), (-1,2))[:-1,:]

    # Find Joint Positions when blobs are obfuscated
    # Assumptions are as follows:
    # Yellow blob will never be completely obfuscated
    # The same blob will not be completely obfuscated in both cameras
    # Iterate over all blobs:
    if self.distance_ratio is not None:
      link_lengths = np.array([2.5,3.5,3]) # link_lengths contains the links with a non-zero length
      link_lengths *= 1/self.distance_ratio # convert to pixel units

      for i in range(1,len(center_info_1)):
        if center_info_1[i,0] == -1 and center_info_2[i,0] == -1:
          sys.exit() # this case when both x and y are unkown should never be true

        # If y position is unknown
        if center_info_1[i,0] == -1:
          # Sets the missing axis' value as a function of the link length, previous joint position, 
          # and known coordinates of the current joint position.
          delta_z = np.abs((center_info_2[i,1] + center_info_1[i-1,1])/2 -  (center_info_2[i-1,1] + center_info_1[i-1,1])/2)
          delta_x = np.abs(center_info_2[i,0] - center_info_2[i-1,0])
          center_info_1[i,0] = np.abs(link_lengths[i-1] - delta_z - delta_x) + center_info_1[i-1,0]
          center_info_1[i,1] = center_info_2[i,1]
          # for testing
          # print("Blob ",i ,"obfuscated found y = ", center_info_1[i,0]) 
          # print("delta z = ", delta_z)
          # print("delta x = ", delta_x)
          # print("yB = ?  yA = ",center_info_1[i-1,0] ) 

        # If x position is unknown
        if center_info_2[i,0] == -1:
          delta_z = np.abs((center_info_1[i,1] + center_info_2[i-1,1])/2 -  (center_info_1[i-1,1] + center_info_2[i-1,1])/2)
          delta_y = np.abs(center_info_1[i,0] - center_info_1[i-1,0])
          center_info_2[i,0] = np.abs(link_lengths[i-1] - delta_z - delta_y) + center_info_2[i-1,0]
          center_info_2[i,1] = center_info_1[i,1]
          # for testing
          # print("Blob ",i ,"obfuscated found x = ", center_info_2[i,0])
          # print("delta z = ", delta_z)
          # print("delta y = ", delta_y)
          # print("xB = ?  xA = ",center_info_2[i-1,0], "zA = ", center_info_1[i,1] )

    for i in range(1,len(center_info_1)):
      if(center_info_1[i,0] != -1 and center_info_2[i,0] != -1):
        self.blob_location[i,0] = center_info_2[i,0] - center_info_2[0,0]
        self.blob_location[i,1] = center_info_1[i,0] - center_info_1[0,0]
        self.blob_location[i,2] = -(center_info_2[i,1] - center_info_2[0,1] + center_info_1[i,1] - center_info_1[0,1])/ 2

    # For testing: red dots placed on non-red blob centers and green dot placed on red blob center
    # yellow blob:
    self.cv_image1[center_info_1[0,1]-1:center_info_1[0,1]+1, center_info_1[0,0]-1:center_info_1[0,0]+1, 0] = 0
    self.cv_image1[center_info_1[0,1]-1:center_info_1[0,1]+1, center_info_1[0,0]-1:center_info_1[0,0]+1, 1] = 0
    self.cv_image1[center_info_1[0,1]-1:center_info_1[0,1]+1, center_info_1[0,0]-1:center_info_1[0,0]+1, 2] = 255

    # blue blob:
    self.cv_image1[center_info_1[1,1]-1:center_info_1[1,1]+1, center_info_1[1,0]-1:center_info_1[1,0]+1, 0] = 0
    self.cv_image1[center_info_1[1,1]-1:center_info_1[1,1]+1, center_info_1[1,0]-1:center_info_1[1,0]+1, 1] = 0
    self.cv_image1[center_info_1[1,1]-1:center_info_1[1,1]+1, center_info_1[1,0]-1:center_info_1[1,0]+1, 2] = 255

    # green blob:
    self.cv_image1[center_info_1[2,1]-1:center_info_1[2,1]+1, center_info_1[2,0]-1:center_info_1[2,0]+1, 0] = 0
    self.cv_image1[center_info_1[2,1]-1:center_info_1[2,1]+1, center_info_1[2,0]-1:center_info_1[2,0]+1, 1] = 0
    self.cv_image1[center_info_1[2,1]-1:center_info_1[2,1]+1, center_info_1[2,0]-1:center_info_1[2,0]+1, 2] = 255

    # red blob:
    self.cv_image1[center_info_1[3,1]-1:center_info_1[3,1]+1, center_info_1[3,0]-1:center_info_1[3,0]+1, 0] = 0
    self.cv_image1[center_info_1[3,1]-1:center_info_1[3,1]+1, center_info_1[3,0]-1:center_info_1[3,0]+1, 1] = 255
    self.cv_image1[center_info_1[3,1]-1:center_info_1[3,1]+1, center_info_1[3,0]-1:center_info_1[3,0]+1, 2] = 0

    # cv2.imshow('Obfuscation', self.cv_image1)

    # if self.distance_ratio is None:
      # meter value for a single unit
    self.distance_ratio = (2.5 / np.linalg.norm(self.blob_location[1,:] - self.blob_location[0,:]) + 
                            3.5 / np.linalg.norm(self.blob_location[2,:] - self.blob_location[1,:]) +
                            3 / np.linalg.norm(self.blob_location[3,:] - self.blob_location[2,:])) / 3

    self.blob_location *= self.distance_ratio

    # Publishing target location
    if self.distance_ratio is not None:
      # Assume target is never obfuscated
      target_info_2 = data.data[-2:] # [x,z] coordinate offrom camera 2
      target_info_1 = detect_target_center(image1) # [y,z] coordinate from camera 2
      self.target_location[0] = target_info_2[0] - center_info_2[0,0] # x coordinate relative to robot base frame
      self.target_location[1] = target_info_1[0] - center_info_1[0,0] # y coordinate relative to robot base frame
      self.target_location[2] = -(target_info_2[1] - center_info_2[0,1] + target_info_1[1] - center_info_1[0,1])/2 # z coordinate relative to robot base frame

      self.target_location *= self.distance_ratio

      self.target_x = Float64()
      self.target_x.data = self.target_location[0]
      self.target_x_publisher.publish(self.target_x)

      self.target_y = Float64()
      self.target_y.data = self.target_location[1]
      self.target_y_publisher.publish(self.target_y)

      self.target_z = Float64()
      self.target_z.data = self.target_location[2]
      self.target_z_publisher.publish(self.target_z)

    self.angle_estimation()
    # print("%0.3f  %0.3f  %0.3f  %0.3f" % tuple(self.joint_angles)) # only looking at the first three decimal places so any noice seen is significant
    
  def angle_estimation(self):
    # Can't easily find joint_angles[0] from blob detection, so assume it is known

    # Find second and third joint angles by analysing forward kinematics equation 
    # for the location of the third blob, which is actually the forth joint
    self.joint_angles[1] = np.arctan2(self.blob_location[2,0] * np.sin(self.joint_angles[0]) 
                            - self.blob_location[2,1] * np.cos(self.joint_angles[0]), 
                            self.blob_location[2,2] - 2.5) 

    self.joint_angles[2] = np.arctan2((self.blob_location[2,0] * np.cos(self.joint_angles[0]) 
                            + self.blob_location[2,1] * np.sin(self.joint_angles[0])) * np.cos(self.joint_angles[1]), 
                            self.blob_location[2,2] - 2.5) 

    # Find the forth joint angle by finding the angle it makes with its unrotated position,
    # then determine the correct sign by checking which has the smaller error
    v1 = self.blob_location[3] - self.blob_location[2]
    v2 = self.blob_location[2] - self.blob_location[1]
    angle = np.arccos(np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2)))

    positive = [self.joint_angles[i] for i in range(3)] + [angle]
    negative = [self.joint_angles[i] for i in range(3)] + [-angle]

    if np.linalg.norm(self.blob_location[3] - self.forward_kinematics(positive)) < \
        np.linalg.norm(self.blob_location[3] - self.forward_kinematics(negative)): 
      self.joint_angles[3] = angle
    else:
      self.joint_angles[3] = -angle
      


  def forward_kinematics(self, angles):
    """
    Matrices used in deriving the forward kinematics
    frame_1_0 = np.array([
      [np.cos(angles[0] + np.pi / 2), 0, np.sin(angles[0] + np.pi / 2), 0],
      [np.sin(angles[0] + np.pi / 2), 0, -np.cos(angles[0] + np.pi / 2), 0],
      [0, 1, 0, 2.5],
      [0, 0, 0, 1]])

    frame_2_1 = np.array([
      [np.cos(angles[1] + np.pi / 2), 0, np.sin(angles[1] + np.pi / 2), 0],
      [np.sin(angles[1] + np.pi / 2), 0, -np.cos(angles[1] + np.pi / 2), 0],
      [0, 1, 0, 0],
      [0, 0, 0, 1]])

    frame_3_2 = np.array([
      [np.cos(angles[2]), 0, -np.sin(angles[2]), 3.5 * np.cos(angles[2])],
      [np.sin(angles[2]), 0, np.cos(angles[2]), 3.5 * np.sin(angles[2])],
      [0, -1, 0, 0],
      [0, 0, 0, 1]])

    frame_4_3 = np.array([
      [np.cos(angles[3]), -np.sin(angles[3]), 0, 3 * np.cos(angles[3])],
      [np.sin(angles[3]), np.cos(angles[3]), 0, 3 * np.sin(angles[3])],
      [0, 0, 1, 0],
      [0, 0, 0, 1]])

    fk_matrix = frame_1_0.dot(frame_2_1.dot(frame_3_2.dot(frame_4_3)))
    r1 = fk_matrix[0:-1, -1]
    """

    return np.array([
      np.sin(angles[0]) * np.sin(angles[1]) * np.cos(angles[2]) * (3.5 + 3 * np.cos(angles[3]))
      + np.cos(angles[0]) * np.sin(angles[2]) * (3.5  + 3 * np.cos(angles[3]))
      + 3 * np.sin(angles[0]) * np.cos(angles[1]) * np.sin(angles[3]), 

      np.sin(angles[0]) * np.sin(angles[2]) * (3.5 + 3 * np.cos(angles[3]))
      - np.cos(angles[0]) * np.sin(angles[1]) * np.cos(angles[2]) * (3.5  + 3 * np.cos(angles[3]))
      - 3 * np.cos(angles[0]) * np.cos(angles[1]) * np.sin(angles[3]), 

      np.cos(angles[1]) * np.cos(angles[2]) * (3.5 + 3 * np.cos(angles[3]))
      - 3 * np.sin(angles[1]) * np.sin(angles[3]) + 2.5  
    ])
  
  def jacobian(self, angles):
    # I really hope this is correct
    return np.array(
      [
        [
          np.cos(angles[0]) * np.sin(angles[1]) * np.cos(angles[2]) * (3.5 + 3 * np.cos(angles[3]))
          - np.sin(angles[0]) * np.sin(angles[2]) * (3.5  + 3 * np.cos(angles[3]))
          + 3 * np.cos(angles[0]) * np.cos(angles[1]) * np.sin(angles[3]), 

          np.sin(angles[0]) * np.cos(angles[1]) * np.cos(angles[2]) * (3.5 + 3 * np.cos(angles[3]))
          - 3 * np.sin(angles[0]) * np.sin(angles[1]) * np.sin(angles[3]), 

          - np.sin(angles[0]) * np.sin(angles[1]) * np.sin(angles[2]) * (3.5 + 3 * np.cos(angles[3]))
          + np.cos(angles[0]) * np.cos(angles[2]) * (3.5  + 3 * np.cos(angles[3])),

          - np.sin(angles[0]) * np.sin(angles[1]) * np.cos(angles[2]) * 3 * np.sin(angles[3])
          + np.cos(angles[0]) * np.sin(angles[2]) * 3 * np.sin(angles[3])
          + 3 * np.sin(angles[0]) * np.cos(angles[1]) * np.cos(angles[3]), 
        ],

        [
          np.cos(angles[0]) * np.sin(angles[2]) * (3.5 + 3 * np.cos(angles[3]))
          + np.sin(angles[0]) * np.sin(angles[1]) * np.cos(angles[2]) * (3.5  + 3 * np.cos(angles[3]))
          + 3 * np.sin(angles[0]) * np.cos(angles[1]) * np.sin(angles[3]),

          - np.cos(angles[0]) * np.cos(angles[1]) * np.cos(angles[2]) * (3.5  + 3 * np.cos(angles[3]))
          + 3 * np.cos(angles[0]) * np.sin(angles[1]) * np.sin(angles[3]), 

          np.sin(angles[0]) * np.cos(angles[2]) * (3.5 + 3 * np.cos(angles[3]))
          + np.cos(angles[0]) * np.sin(angles[1]) * np.sin(angles[2]) * (3.5  + 3 * np.cos(angles[3])),

          - np.sin(angles[0]) * np.sin(angles[2]) * 3 * np.sin(angles[3])
          + np.cos(angles[0]) * np.sin(angles[1]) * np.cos(angles[2]) * 3 * np.sin(angles[3])
          - 3 * np.cos(angles[0]) * np.cos(angles[1]) * np.cos(angles[3]), 
        ],

        [0, 
          - np.sin(angles[1]) * np.cos(angles[2]) * (3.5 + 3 * np.cos(angles[3])) - 3 * np.cos(angles[1]) * np.sin(angles[3]),
          - np.cos(angles[1]) * np.sin(angles[2]) * (3.5 + 3 * np.cos(angles[3])),
          - np.cos(angles[1]) * np.cos(angles[2]) * 3 * np.sin(angles[3]) - 3 * np.sin(angles[1]) * np.cos(angles[3])
        ]
      ]
    )

    

def rgb_normalize(image):
  # Use RGB normalization to handle the varying lighting
  rgb_norm_image = np.zeros(np.shape(image),np.float64)

  b,g,r = cv2.split(image)

  rgb_norm_image[:,:,0] = np.divide(b, b + g + 1.2*r + np.ones(np.shape(r))) * 255 # Add one to each element to deal with division by 0
  rgb_norm_image[:,:,1] = np.divide(g, b + g + 1.2*r + np.ones(np.shape(r))) * 255
  rgb_norm_image[:,:,2] = np.divide(r, b + g + 1.2*r + np.ones(np.shape(r))) * 255

  rgb_norm_image = cv2.convertScaleAbs(rgb_norm_image)

  # imshow used for testing purposes:
  # cv2.imshow('normalization', rgb_norm_image)
  # cv2.waitKey(1)

  return rgb_norm_image


def detect_color_center(image, lower_bound, upper_bound):
  # Thresholding and removing noise
  thresholded = cv2.inRange(image, lower_bound, upper_bound)
  thresholded = cv2.erode(thresholded, np.ones(3, np.uint8))
  thresholded = cv2.dilate(thresholded, np.ones(3, np.uint8))
  # Finding the center point
  try:
    moments = cv2.moments(thresholded)
    cx = int(moments['m10']/moments['m00'])
    cy = int(moments['m01']/moments['m00'])
    return np.array([cx,cy])
  except:
    return np.array([-1,-1])


def detect_yellow_center(image):
  # Color boundaries
  yellow_lower = np.array([0,100,100])
  yellow_upper = np.array([10,150,150])
  return detect_color_center(image, yellow_lower, yellow_upper)

def detect_blue_center(image):
  # Color boundaries
  blue_lower = np.array([200,0,0])
  blue_upper = np.array([255,10,10])
  return detect_color_center(image, blue_lower, blue_upper)
    
def detect_green_center(image):
  # Color boundaries
  green_lower = np.array([0,200,0])
  green_upper = np.array([10,255,10])
  return detect_color_center(image, green_lower, green_upper)

def detect_red_center(image):
  # Color boundaries
  red_lower = np.array([0,0,180])
  red_upper = np.array([10,10,255])
  return detect_color_center(image, red_lower, red_upper)

def threshold_orange(image):
  # Color boundaries
  orange_lower = np.array([0,50,100])
  orange_upper = np.array([10,100,175])
  # Thresholding and removing noise
  thresholded = cv2.inRange(image, orange_lower, orange_upper)
  thresholded = cv2.erode(thresholded, np.ones(3, np.uint8))
  thresholded = cv2.dilate(thresholded, np.ones(3, np.uint8))

  # cv2.imshow("all", thresholded)
  # cv2.waitKey(1)

  return thresholded

def detect_target_center(image):
  thresholded = threshold_orange(image)

  thresholded_copy = thresholded

  where_orange = np.argwhere(thresholded)
  y_max = np.max(where_orange[:,0]) + 20
  y_min = np.min(where_orange[:,0]) - 20
  x_max = np.max(where_orange[:,1]) + 20
  x_min = np.min(where_orange[:,1]) - 20

  # Crop the thresholded image to the region containing both orange objects
  thresholded_cropped = thresholded[y_min : y_max, x_min : x_max]

  # Run from the catkin_ws directory
  template_path = os.path.realpath("src/ivr_assignment/src/target.png")
  template = cv2.imread(template_path, 0)

  # Resize the template to capture a larger area
  resize_dims = (int(np.round(1.3 * (template.shape[0]))), int(np.round(1.3 * (template.shape[1]))))
  # template = cv2.resize(template, resize_dims)

  # Distance Transform Matrix
  dists = cv2.distanceTransform(cv2.bitwise_not(thresholded_cropped), cv2.DIST_L2, 0)
  
  step_size = 3
  y_range = round((thresholded_cropped.shape[0] - template.shape[0]) / step_size)
  x_range = round((thresholded_cropped.shape[1] - template.shape[1]) / step_size)

  chamfer_scores = np.zeros((y_range,x_range))
  # Iterate over y positions
  for i in range(y_range):
    # Iterate over x positions
    for j in range(x_range):
      # Multiply the relevent area of the distance transformed image with the template
      # translated to the current iteration's region
      chamfer_scores[i,j] = np.sum(dists[i*step_size:template.shape[0] + i*step_size, j*step_size :template.shape[1] + j*step_size] * template)
  # The translation region which yields the lowest chamfer score is the region containing the target
  best_translation = np.argmin(chamfer_scores)
  # print("Best Chamfer Score: ", np.min(chamfer_scores))


  # print(best_translation)

  # Values corresponding to the region of the target in the cropped and thresholded image
  y_relative_min = (best_translation % y_range) * step_size
  y_relative_max = (best_translation % y_range) * step_size + template.shape[0]
  x_relative_min = (best_translation % x_range) * step_size
  x_relative_max = (best_translation % x_range) * step_size + template.shape[1]

  target = thresholded_cropped[y_relative_min:y_relative_max, x_relative_min:x_relative_max]

  # Values corresponding to the region of the target in the full-sized (non-cropped) thresholded image
  y_final_min = y_relative_min + y_min - 5
  y_final_max = y_relative_max + y_min + 5
  x_final_min = x_relative_min + x_min - 5
  x_final_max = x_relative_max + x_min + 5

  target_region = np.zeros(thresholded.shape, dtype = np.uint8)
  target_region[y_final_min:y_final_max, x_final_min:x_final_max] = 255
  thresholded = cv2.bitwise_and(thresholded, target_region)

  # Region for testing
  region1 = np.zeros(thresholded.shape)
  region1[y_min:y_max, x_min:x_max] = 255
  region1[y_final_min:y_final_max, x_final_min:x_final_max] = 0


  thresholded_copy[y_final_min: y_final_max, x_final_min - 1: x_final_min + 1] = 200
  thresholded_copy[y_final_min: y_final_max, x_final_max - 1: x_final_max + 1] = 200
  thresholded_copy[y_final_min - 1: y_final_min + 1, x_final_min: x_final_max] = 200
  thresholded_copy[y_final_max - 1: y_final_max + 1, x_final_min: x_final_max] = 200

  thresholded_copy[y_min: y_max, x_min - 1: x_min + 1] = 200
  thresholded_copy[y_min: y_max, x_max - 1: x_max + 1] = 200
  thresholded_copy[y_min - 1: y_min + 1, x_min: x_max] = 200
  thresholded_copy[y_max - 1: y_max + 1, x_min: x_max] = 200



  # Testing purposes
  cv2.imshow("distance_transform", dists)
  cv2.imshow("template", template)
  # cv2.imshow("target", target)
  # cv2.imshow("thresholded cropped", thresholded_cropped)
  # cv2.imshow("thresholded", thresholded)
  cv2.imshow("thresholded copy", thresholded_copy)
  # cv2.imshow("test_region", region1)
  cv2.waitKey(1)

  # Finding the center point
  try:
    moments = cv2.moments(thresholded)
    cx = int(moments['m10']/moments['m00'])
    cy = int(moments['m01']/moments['m00'])
    return np.array([cx,cy])
  except:
    print("Best Chamfer Score: ", np.min(chamfer_scores))
    moments = cv2.moments(threshold_orange(image)) 
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

