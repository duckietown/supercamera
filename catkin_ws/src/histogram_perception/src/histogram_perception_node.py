#!/usr/bin/env python

import rospy
from histogram_perception import HistogramPerception


def on_shutdown():
  rospy.loginfo("[%s] Shutting down." % rospy.get_name())

def main():
  # Initialize the node with rospy
  rospy.init_node('histogram_perception_node', anonymous=False)
  # Create the DaguCar object
  histogramPerception = HistogramPerception()
  # Setup proper shutdown behavior
  rospy.on_shutdown(on_shutdown)
  # Keep it spinning to keep the node alive
  rospy.spin()

if __name__ == '__main__':
  main()
