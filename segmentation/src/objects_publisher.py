#!/usr/bin/env python
"""Skeleton code for Lab 6
Course: EECS C106A, Fall 2019
Author: Amay Saxena

This file implements a ROS node that subscribes to topics for RGB images,
pointclouds, and camera calibration info, and uses the functions you
implemented to publish a segmented pointcloud to the topic /segmented_points.

Once you are confident in your implementation in image_segmentation.py and
pointcloud_segmentation.py, run this file to begin publishing a segmented
pointcloud.
"""
import rospy
from mv.msg import Pickup
from main import PointcloudProcess
def talker(objects, transforms):
    #Run this program as a new node in the ROS computation graph 
    #called /talker.
    rospy.init_node('object_listener', anonymous=True)

    #Create an instance of the rospy.Publisher object which we can 
    #use to publish messages to a topic. This publisher publishes 
    #messages of type std_msgs/String to the topic /chatter_talk
    pub = rospy.Publisher('objects', Pickup, queue_size=10)

    # Create a timer object that will sleep long enough to result in
    # a 10Hz publishing rate
    r = rospy.Rate(10) # 10hz

    msg = Pickup()
    msg.objects, msg.transforms = objects, transforms
    # Publish our string to the 'chatter_talk' topic
    pub.publish(msg, rospy.get_time())

if __name__ == '__main__':
	while True:
		objects = PointcloudProcess.objects
		poses = PointcloudProcess.poses
		print(objects)
		talker(objects, poses)
