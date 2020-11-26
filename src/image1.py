#!/usr/bin/env python3

# IMPORTANT:
# Make sure to run "image1.py" and "image2.py" from the catkin_make folder
# with this repository's contents inside of the catkin_make/src directory!

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
    # intialize a subsriber to recieve the image seen by camera 2
    self.image_sub2 = rospy.Subscriber("/camera2/robot/image_raw", Image, self.callback2)
    # initialize the bridge between openCV and ROS
    self.bridge = CvBridge()
    # initialize publishers to send inputs to the robot's joints
    self.joint1_pub = rospy.Publisher("/robot/joint1_position_controller/command", Float64, queue_size=10)
    self.joint2_pub = rospy.Publisher("/robot/joint2_position_controller/command", Float64, queue_size=10)
    self.joint3_pub = rospy.Publisher("/robot/joint3_position_controller/command", Float64, queue_size=10)
    self.joint4_pub = rospy.Publisher("/robot/joint4_position_controller/command", Float64, queue_size=10)

    self.start_time = rospy.get_time()

    self.previous_time = rospy.get_time()
    self.previous_error = 0

    # initialize subscribers to recieve blob centers from image2
    self.blob_subscriber = rospy.Subscriber("/camera2/blob_data", Int64MultiArray, self.im2_update)
    self.blob_location = np.zeros((4,3))
    self.joint_angles = np.zeros(4)

    # initialize a publisher to publish the target's 3d location
    self.target_x_publisher = rospy.Publisher("/target/x_location", Float64, queue_size = 1)
    self.target_y_publisher = rospy.Publisher("/target/y_location", Float64, queue_size = 1)
    self.target_z_publisher = rospy.Publisher("/target/z_location", Float64, queue_size = 1)
    self.target_location = np.zeros(3)

    # initialize a publisher to publish the estimated joint angles
    self.estimated_joint2_publisher = rospy.Publisher("/joint2/estimated_angle", Float64, queue_size = 1)
    self.estimated_joint3_publisher = rospy.Publisher("/joint3/estimated_angle", Float64, queue_size = 1)
    self.estimated_joint4_publisher = rospy.Publisher("/joint4/estimated_angle", Float64, queue_size = 1)
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

    # Hardcoded blue and yellow joint positions which are both fixed
    # Blue Joint
    #self.cv_image1[469:471, 398:400, 0] = 0
    #self.cv_image1[469:471, 398:400, 1] = 0
    #self.cv_image1[469:471, 398:400, 2] = 255

    # Yellow Joint
    #self.cv_image1[532:534, 398:400, 0] = 0
    #self.cv_image1[532:534, 398:400, 1] = 255
    #self.cv_image1[532:534, 398:400, 2] = 0

    cv2.imshow('window1', self.cv_image1)
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
    
    # self.test_FK()

  def callback2(self,data):
    try:
      self.cv_image2 = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    #cv2.imshow("camera2_image1", self.cv_image2)
    #cv2.waitKey(1)

    self.distance_ratio = 0.038714 # from using blob detection with coloured joints

    center_info_1 = np.zeros((4,2)) # info from camera 1
    center_info_2 = np.zeros((4,2)) #  info from camera 2

    # Hard code yellow blob position from previous tasks with non black joints
    center_info_1[0,0] = 399
    center_info_1[0,1] = 533
    center_info_2[0,0] = 399
    center_info_2[0,1] = 533
    self.blob_location[0,:] = np.array([0, 0, 0]) # location relative to robot base frame in meters

    # Hard code blue blow position from previous tasks with non black  joints
    center_info_1[1,0] = 399
    center_info_1[1,1] = 470
    center_info_2[1,0] = 399
    center_info_2[1,1] = 470
    self.blob_location[1,:] = np.array([0, 0, 2.5]) # location relative to robot base frame in meters

    angle_l2_cam1 = extension_chamfer_l2(self.cv_image1, self.distance_ratio, center_info_1[1,0], center_info_1[1,1]) # theta1
    angle_l2_cam2 = extension_chamfer_l2(self.cv_image2, self.distance_ratio, center_info_2[1,0], center_info_2[1,1]) # theta2

    # Length of link 2 from camera 1
    length2_info_1 = np.sqrt( (3.5 ** 2) / ( ( np.cos(angle_l2_cam1) / np.cos(angle_l2_cam2) ) ** 2 + np.sin(angle_l2_cam1) ** 2  ) )
    # Length of link 2 from camera 2
    length2_info_2 = length2_info_1 * ( np.cos(angle_l2_cam1) / np.cos(angle_l2_cam2) )

    # where these coordinates are relative to blob 2
    relative_x = -length2_info_2 * np.sin(angle_l2_cam2) # negative due to orientation of the axis
    relative_y = -length2_info_1 * np.sin(angle_l2_cam1) # negative due to orientation of the axis
    relative_z = length2_info_1 * np.cos(angle_l2_cam1) # we assumed z is the same from both cameras
    
    self.blob_location[2,0] = relative_x + self.blob_location[1,0]
    self.blob_location[2,1] = relative_y + self.blob_location[1,1]
    self.blob_location[2,2] = self.blob_location[1,2] - relative_z

    center_info_1[2,0] = int(round((relative_y/self.distance_ratio) + center_info_1[1,0]))
    center_info_1[2,1] = int(round(center_info_1[1,1] - (relative_z/self.distance_ratio)))
    center_info_2[2,0] = int(round((relative_x/self.distance_ratio) + center_info_2[1,0]))
    center_info_2[2,1] = int(round(center_info_1[2,1]))

    
    angle_l3_cam1 = extension_chamfer_l3(self.cv_image1, self.distance_ratio, center_info_1[2,0], center_info_1[2,1], angle_l2_cam1)
    angle_l3_cam1 = angle_l3_cam1 + angle_l2_cam1 # add on previous joint angle to give angle from the vertical line
    angle_l3_cam2 = extension_chamfer_l3(self.cv_image2, self.distance_ratio, center_info_2[2,0], center_info_2[2,1], angle_l2_cam2)
    angle_l3_cam2 = angle_l3_cam2 + angle_l2_cam2

    #print(angle_l3_cam1, angle_l3_cam2)

    # Length of link 3 from camera 1
    length3_info_1 = np.sqrt( (3 ** 2) / ( ( np.cos(angle_l3_cam1) / np.cos(angle_l3_cam2) ) ** 2 + np.sin(angle_l3_cam1) ** 2  ) )
    # Length of link 3 from camera 2
    length3_info_2 = length3_info_1 * ( np.cos(angle_l3_cam1) / np.cos(angle_l3_cam2) )

    # where these coordinates are relative to blob 3
    relative_x = -length3_info_2 * np.sin(angle_l3_cam2) # negative due to orientation of the axis
    relative_y = -length3_info_1 * np.sin(angle_l3_cam1) # negative due to orientation of the axis
    relative_z = length3_info_1 * np.cos(angle_l3_cam1) # we assumed z is the same from both cameras

    self.blob_location[3,0] = relative_x + self.blob_location[2,0]
    self.blob_location[3,1] = relative_y + self.blob_location[2,1]
    self.blob_location[3,2] = self.blob_location[2,2] - relative_z

    center_info_1[3,0] = int(round((relative_y/self.distance_ratio) + center_info_1[2,0]))
    center_info_1[3,1] = int(round(center_info_1[2,1] - (relative_z/self.distance_ratio)))
    center_info_2[3,0] = int(round((relative_x/self.distance_ratio) + center_info_2[2,0]))
    center_info_2[3,1] = int(round(center_info_1[3,1]))

    # Green dot on estimated position for end effector in both cameras to test
    #self.cv_image1[int(center_info_1[3,1] - 1):int(center_info_1[3,1] + 1), int(center_info_1[3,0] - 1):int(center_info_1[3,0] + 1), 0] = 0
    #self.cv_image1[int(center_info_1[3,1] - 1):int(center_info_1[3,1] + 1), int(center_info_1[3,0] - 1):int(center_info_1[3,0] + 1), 1] = 255
    #self.cv_image1[int(center_info_1[3,1] - 1):int(center_info_1[3,1] + 1), int(center_info_1[3,0] - 1):int(center_info_1[3,0] + 1), 2] = 0

    #self.cv_image2[int(center_info_2[3,1] - 1):int(center_info_2[3,1] + 1), int(center_info_2[3,0] - 1):int(center_info_2[3,0] + 1), 0] = 0
    #self.cv_image2[int(center_info_2[3,1] - 1):int(center_info_2[3,1] + 1), int(center_info_2[3,0] - 1):int(center_info_2[3,0] + 1), 1] = 255
    #self.cv_image2[int(center_info_2[3,1] - 1):int(center_info_2[3,1] + 1), int(center_info_2[3,0] - 1):int(center_info_2[3,0] + 1), 2] = 0
    cv2.imshow("estimation1", self.cv_image1)
    cv2.imshow("estimation2", self.cv_image2)
    cv2.waitKey(1)

    self.angle_estimation()

    # Publish estimated joint angles for joints 2,3,4
    self.estimated_joint2 = Float64()
    self.estimated_joint2.data = self.joint_angles[1]
    self.estimated_joint2_publisher.publish(self.estimated_joint2)

    self.estimated_joint3 = Float64()
    self.estimated_joint3.data = self.joint_angles[2]
    self.estimated_joint3_publisher.publish(self.estimated_joint3)

    self.estimated_joint4 = Float64()
    self.estimated_joint4.data = self.joint_angles[3]
    self.estimated_joint4_publisher.publish(self.estimated_joint4)

  #def im2_update(self,data):

  # IMPORTANT:
  # ALL CODE FROM IM2_UPDATE WAS DELETED FOR THE QUESTION FROM SECTION 4.
  # THIS CODE IS REQUIRED TO GET RESULTS FOR PREVIOUS SECTIONS IN THE ASSIGNMENT.
  # SEE THE BRANCH NAMED "current" FOR THIS FUNCTIONALITY.

  def angle_estimation(self):
    # Can't easily find joint_angles[0] from blob detection, so assume it is known

    # Find second and third joint angles by analysing forward kinematics equation 
    # for the location of the third blob, which is actually the forth joint
    x1 = self.blob_location[2,0] * np.sin(self.joint_angles[0]) - self.blob_location[2,1] * np.cos(self.joint_angles[0])
    x2 = self.blob_location[2,2] - self.blob_location[1,2]

    # print(x1,x2, self.blob_location[2])

    # Helps minimise the influence of small errors
    if np.abs(x1) < 0.01:
      x1 = 0
    if np.abs(x2) < 0.01:
      x2 = 0
    if x2 < 0:
      x1 *= -1
      x2 *= -1

    self.joint_angles[1] = np.arctan2(x1, x2)
                             
    x1 = (self.blob_location[2,0] * np.cos(self.joint_angles[0]) + self.blob_location[2,1] * np.sin(self.joint_angles[0])) * np.cos(self.joint_angles[1])
    x2 = self.blob_location[2,2] - self.blob_location[1,2]
    if np.abs(x1) < 0.01:
      x1 = 0
    if np.abs(x2) < 0.01:
      x2 = 0
    if x2 < 0:
      x1 *= -1
      x2 *= -1

    self.joint_angles[2] = np.arctan2(x1, x2) 

    # Find the forth joint angle by finding the angle it makes with its unrotated position,
    # then determine the correct sign by checking which has the smaller error
    x1 = self.blob_location[3] - self.blob_location[2]
    x2 = self.blob_location[2] - self.blob_location[1]
    angle = np.arccos(np.dot(x1, x2) / (np.linalg.norm(x1) * np.linalg.norm(x2)))

    positive = [self.joint_angles[i] for i in range(3)] + [angle]
    negative = [self.joint_angles[i] for i in range(3)] + [-angle]

    if np.linalg.norm(self.blob_location[3] - self.forward_kinematics(positive)) < np.linalg.norm(self.blob_location[3] - self.forward_kinematics(negative)): 
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
          - np.cos(angles[0]) * np.sin(angles[2]) * 3 * np.sin(angles[3])
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

  def test_FK(self):
    # q = [np.pi/2, np.pi/2, np.pi/2, -1 * np.pi/2] # first test
    # q = [-1 * np.pi/2, -1 * np.pi/2, -1 * np.pi/2, np.pi/2] # second test
    # q = [np.pi/20, np.pi/20, np.pi/20, np.pi/20] # third test
    # q = [-1 * np.pi/20, -1 * np.pi/20, -1 * np.pi/20, -1 * np.pi/20] # fourth test
    # q = [-1 * np.pi/4, np.pi/4, -1 * np.pi/4, np.pi/4] # fifth test
    # q = [np.pi/4, -1 * np.pi/4, np.pi/4, -1 * np.pi/4] # sixth test
    # q = [-1 * np.pi/2, np.pi/3, np.pi/4, -1 * np.pi/5] # seventh test
    # q = [np.pi/5, -1 * np.pi/4, -1 * np.pi/3, np.pi/2] # eigth test
    # q = [np.pi/3, np.pi/30, np.pi/3, np.pi/30] # ninth test
    q = [np.pi/2.5, np.pi/25, np.pi/3.5, np.pi/35] # tenth test

    print("Calculated Position: ", self.forward_kinematics(q))

    # Publish the joint inputs used for the current test
    self.joint1 = Float64()
    self.joint1.data = q[0]
    self.joint1_pub.publish(self.joint1)

    self.joint2 = Float64()
    self.joint2.data = q[1]
    self.joint2_pub.publish(self.joint2)

    self.joint3 = Float64()
    self.joint3.data = q[2]
    self.joint3_pub.publish(self.joint3)

    self.joint4 = Float64()
    self.joint4.data = q[3]
    self.joint4_pub.publish(self.joint4)
    
    print("Estimated Position: ", self.blob_location[3,:])

  def closed_loop_control(self, target_position, angles):
    k_d = np.diag(np.full(3, 0.1))
    k_p = np.diag(np.full(3, 4))

    time = rospy.get_time()
    dt = time - self.previous_time
    self.previous_time = time 

    error = target_position - self.forward_kinematics(angles)
    dE = (error - self.previous_error) / dt 
    self.previous_error = error

    jacobian = self.jacobian(angles)[:,1:] # choose to keep first joint fixed
    
    # just np.linalg.pinv(jacobian) is giving me an error, not sure why
    jacobian_inverse = np.linalg.pinv(jacobian.dot(jacobian.T)).dot(jacobian.T)

    
    dq = jacobian_inverse.dot(k_d.dot(dE) + k_p.dot(error))

    new_angles = angles[1:] + dq * dt
    return new_angles

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
  # cv2.imshow("distance_transform", dists)
  # cv2.imshow("template", template)
  # cv2.imshow("target", target)
  # cv2.imshow("thresholded cropped", thresholded_cropped)
  # cv2.imshow("thresholded", thresholded)
  # cv2.imshow("thresholded copy", thresholded_copy)
  # cv2.imshow("test_region", region1)
  cv2.waitKey(1)

  # Finding the center point
  try:
    moments = cv2.moments(thresholded)
    cx = int(moments['m10']/moments['m00'])
    cy = int(moments['m01']/moments['m00'])
    return np.array([cx,cy])
  except:
    moments = cv2.moments(threshold_orange(image)) 
    cx = int(moments['m10']/moments['m00'])
    cy = int(moments['m01']/moments['m00'])
    return np.array([cx,cy])

# Pass center of the blue joint as cx and cy
def extension_chamfer_l2(image, distance_ratio, cx, cy):
  thresholded = cv2.inRange(image, (0,0,0), (10,10,10))
  thresholded = cv2.erode(thresholded, np.ones(3, np.uint8))
  thresholded = cv2.dilate(thresholded, np.ones(3, np.uint8))

  pixel_link_length = int(round(3.5/ distance_ratio)) # link length in pixels
  y_min = int(cy - pixel_link_length)
  y_max = int(cy + pixel_link_length)
  x_min = int(cx - pixel_link_length)
  x_max = int(cx + pixel_link_length)
  cropped_thresholded = thresholded[y_min:y_max, x_min:x_max]

  dists = cv2.distanceTransform(cv2.bitwise_not(cropped_thresholded), cv2.DIST_L2, 0)

  template_path = os.path.realpath("src/ivr_assignment/src/link2_template.png")
  template = cv2.imread(template_path, 0)

  # modified_template is the same shape as the cropped and thresholded region
  # but it contains the template link at the point where the link should be in the
  # cropped and threshold image if the joint angle was 0
  modified_template = np.zeros((y_max - y_min, x_max - x_min))
  y_min_template = int(round(modified_template.shape[0]/2)) - template.shape[0]
  y_max_template = int(round(modified_template.shape[0]/2))
  x_min_template = int(round(modified_template.shape[1]/2) - np.floor(template.shape[1]/2))
  x_max_template = int(round(modified_template.shape[1]/2) + np.ceil(template.shape[1]/2))
  modified_template[y_min_template:y_max_template, x_min_template:x_max_template] = template

  #cv2.imshow("modified_template", modified_template)
  #cv2.waitKey(1)

  angle_range = np.arange(-150, 151, 1)
  chamfer_scores = np.zeros(angle_range.shape[0])
  for i in np.arange(-150,151,1):
    # Note that rotating with the modified template means that the link will not be resized so it will not
    # become thicker than it really is when rotated.
    rotation_matrix = cv2.getRotationMatrix2D((int(round(modified_template.shape[1]/2)), int(round(modified_template.shape[0]/2))), i, 1)
    rotated_mod_template = cv2.warpAffine(modified_template, rotation_matrix, (dists.shape[1], dists.shape[0]))

    chamfer_score = np.sum(rotated_mod_template * dists)
    chamfer_scores[i+150] = chamfer_score

  best_chamfer_score = np.min(chamfer_scores)
  best_angle = ((np.argmin(chamfer_scores)-150)*np.pi)/180 # as radians
  #print("Best chamfer score = ", best_chamfer_score)
  #print("Best Angle = ", best_angle)

  #rotation_matrix = cv2.getRotationMatrix2D((int(round(modified_template.shape[1]/2)), int(round(modified_template.shape[0]/2))), (best_angle*180)/np.pi, 1)
  #best_rotated_mod_template = cv2.warpAffine(modified_template, rotation_matrix, (dists.shape[1], dists.shape[0]))
  #cv2.imshow("best_rotated_mod_template", best_rotated_mod_template)
  #cv2.imshow("cropped_thresholded", cropped_thresholded)
  #cv2.waitKey(1)

  return best_angle

# Pass center of the 3rd blob as cx and cy aswel as the angle of the previous link
# from the same camera (AS RADIANS)
def extension_chamfer_l3(image, distance_ratio, cx, cy, previous_angle):
  thresholded = cv2.inRange(image, (0,0,0), (10,10,10))
  thresholded = cv2.erode(thresholded, np.ones(3, np.uint8))
  thresholded = cv2.dilate(thresholded, np.ones(3, np.uint8))

  pixel_link_length = int(round(3/ distance_ratio)) # link length in pixels
  y_min = int(cy - pixel_link_length)
  y_max = int(cy + pixel_link_length)
  x_min = int(cx - pixel_link_length)
  x_max = int(cx + pixel_link_length)
  cropped_thresholded = thresholded[y_min:y_max, x_min:x_max]

  dists = cv2.distanceTransform(cv2.bitwise_not(cropped_thresholded), cv2.DIST_L2, 0)

  template_path = os.path.realpath("src/ivr_assignment/src/link3_template.png")
  template = cv2.imread(template_path, 0)

  # modified_template is the same shape as the cropped and thresholded region
  # but it contains the template link at the point where the link should be in the
  # cropped and threshold image if the joint angle was 0
  modified_template = np.zeros((y_max - y_min, x_max - x_min))
  y_min_template = int(round(modified_template.shape[0]/2)) - template.shape[0]
  y_max_template = int(round(modified_template.shape[0]/2))
  x_min_template = int(round(modified_template.shape[1]/2) - np.floor(template.shape[1]/2))
  x_max_template = int(round(modified_template.shape[1]/2) + np.ceil(template.shape[1]/2))
  modified_template[y_min_template:y_max_template, x_min_template:x_max_template] = template

  # Start with the link template rotated in the angle of the previous joint
  rotation_matrix = cv2.getRotationMatrix2D((int(round(modified_template.shape[1]/2)), int(round(modified_template.shape[0]/2))), (previous_angle*180)/np.pi, 1)
  modified_template = cv2.warpAffine(modified_template, rotation_matrix, (dists.shape[1], dists.shape[0]))

  #cv2.imshow("modified_template", modified_template)
  #cv2.waitKey(1)

  angle_range = np.arange(-150, 151, 1)
  chamfer_scores = np.zeros(angle_range.shape[0])
  for i in np.arange(-150,151,1):
    # Note that rotating with the modified template means that the link will not be resized so it will not
    # become thicker than it really is when rotated.
    rotation_matrix = cv2.getRotationMatrix2D((int(round(modified_template.shape[1]/2)), int(round(modified_template.shape[0]/2))), i, 1)
    rotated_mod_template = cv2.warpAffine(modified_template, rotation_matrix, (dists.shape[1], dists.shape[0]))

    chamfer_score = np.sum(rotated_mod_template * dists)
    chamfer_scores[i+150] = chamfer_score

  best_chamfer_score = np.min(chamfer_scores)
  best_angle = ((np.argmin(chamfer_scores)-150)*np.pi)/180 # as radians
  #print("Best chamfer score = ", best_chamfer_score)
  #print("Best Angle = ", best_angle)

  #rotation_matrix = cv2.getRotationMatrix2D((int(round(modified_template.shape[1]/2)), int(round(modified_template.shape[0]/2))), (best_angle*180)/np.pi, 1)
  #best_rotated_mod_template = cv2.warpAffine(modified_template, rotation_matrix, (dists.shape[1], dists.shape[0]))
  #cv2.imshow("best_rotated_mod_template", best_rotated_mod_template)
  #cv2.imshow("cropped_thresholded", cropped_thresholded)
  #cv2.waitKey(1)

  return best_angle


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

