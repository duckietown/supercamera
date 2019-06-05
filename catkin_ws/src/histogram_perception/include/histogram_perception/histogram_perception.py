#!/usr/bin/env python

import os
import rospy
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

encoding_to_order = {
  "rgb8": [0, 1, 2],
  "rgba8": [0, 1, 2],
  "rgb16": [0, 1, 2],
  "rgba16": [0, 1, 2],
  "bgr8": [2, 1, 0],
  "bgra8": [2, 1, 0],
  "bgr16": [2, 1, 0],
  "bgra16": [2, 1, 0],
  "mono8": [0],
  "mono16": [0]
}

def calculate_window(length, nums):
  return list(range(0, (length // nums) * nums + 1, length // nums))


class HistogramPerception(object):

  def __init__(self):
    # get node name and vehicle name
    self.node_name = rospy.get_name()
    self.veh_name = os.environ['VEHICLE_NAME']
    self.bridge = CvBridge()

    # setup parameters
    self.num_buckets = self.setup_parameter("~num_buckets", 5)

    # Setup the publisher and subscriber
    self.sub_camera_image = rospy.Subscriber('~image', Image, self.cb_histogram_perception)
    self.pub_histogram_perception = rospy.Publisher('~histogram', String, queue_size=1)

  def cb_histogram_perception(self, image_msg):
    image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding="passthrough")

    height, width, _ = image.shape
    histogram = np.zeros((self.num_buckets, self.num_buckets, 3))

    height_windows = calculate_window(height, self.num_buckets)
    width_windows = calculate_window(width, self.num_buckets)
    for i, (height_start, height_end) in enumerate(zip(height_windows[0:-1], height_windows[1:])):
      for j, (width_start, width_end) in enumerate(zip(width_windows[0:-1], width_windows[1:])):
        color_avg = np.average(image[height_start:height_end, width_start:width_end, :], axis=(0,1))
        color_avg = color_avg[encoding_to_order[image_msg.encoding]]
        histogram[i, j, :] = color_avg
    # turn numpy array into a comma-separated string of uint8 numbers
    data = histogram.flatten().astype(np.uint8).tolist()
    data_str = ','.join([str(self.num_buckets), str(self.num_buckets), '3']) + ':' + \
      ','.join([str(v) for v in data])
    # turn string into a ROS message and publish it
    msg = String(data=data_str)
    self.pub_histogram_perception.publish(msg)

  def setup_parameter(self, param_name, default_value):
    value = rospy.get_param(param_name, default_value)
    # Write to parameter server for transparency
    rospy.set_param(param_name, value)
    rospy.loginfo("[%s] %s = %s " % (self.node_name, param_name, value))
    return value
