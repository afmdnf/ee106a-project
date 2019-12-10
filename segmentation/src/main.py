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

from __future__ import print_function
from collections import deque

import rospy
import message_filters
import ros_numpy
import tf
import tf2_ros
import geometry_msgs
from std_msgs.msg import Header

from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from mv.msg import Pickup

import numpy as np
import cv2

from cv_bridge import CvBridge
from sklearn.decomposition import PCA

from image_segmentation import segment_image, show_image
from pointcloud_segmentation import segment_pointcloud
from sensor_msgs import point_cloud2
import os
import traceback
from mv.msg import StringArray
#from objects_publisher import talker

pca = PCA(3)

def get_camera_matrix(camera_info_msg):
    return np.array(camera_info_msg.K).reshape((3,3))

def isolate_objects_of_interest(points, image, cam_matrix, trans, rot):
    segmented_image = segment_image(image)
    cloudList = []
    for mask in segmented_image:
        cloud = segment_pointcloud(np.copy(points), mask, cam_matrix, trans, rot)
        cloudList.append(cloud)
    return cloudList

def numpy_to_pc2_msg(points):
    return ros_numpy.msgify(PointCloud2, points, stamp=rospy.Time.now(),
        frame_id='camera_depth_optical_frame')

def pca_object(points):
    global pca
    point_array = np.transpose(np.array([[p[0] for p in points],
                                         [p[1] for p in points],
                                         [p[2] for p in points]]))
    if point_array.shape[1] < 3 or point_array.shape[0] < 25:
        if point_array.shape[0] < 25:
            print("Not Enough Points")
        return [0, 0, 0]
    try:
        pca.fit(point_array)
        sigma = np.array(pca.singular_values_)    
        return np.array(sigma / sigma[0])
    except Exception as e:
        print(e)
        return [0, 0, 0]

def classify_object(sigma):
    if sigma[0] != 1:
        return 0

    if (sigma[1] > 0.75 and sigma[2] < 0.3):
        # plate = 1
        return 1
    # elif (sigma[1] > 0.35 and sigma[2] > 0.25):
        # return 'cup'
    elif (sigma[1] < 0.7 and sigma[2] < 0.3):
        # spoon = 2
        return 2
    else:
        # cup = 3
        return 3
    return 0    

def align_axes(points, object_num):
    global pca
    try:
        point_array = np.transpose(np.array([[p[0] for p in points],
                                             [p[1] for p in points],
                                             [p[2] for p in points]]))
        com = np.median(point_array, 0)
        rot_mat = np.transpose(pca.components_)
        rot_mat = np.hstack((rot_mat, np.zeros((3, 1))))
        rot_mat = np.vstack((rot_mat, [0, 0, 0, 1]))
        rot_quat = tf.transformations.quaternion_from_matrix(rot_mat)
        rot_quat = rot_quat / np.linalg.norm(rot_quat)

        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "camera_depth_optical_frame"
        t.child_frame_id = "object_center_of_mass_" + str(object_num)
        t.transform.translation.x = com[0]
        t.transform.translation.y = com[1]
        t.transform.translation.z = com[2]

        t.transform.rotation.x = rot_quat[0]
        t.transform.rotation.y = rot_quat[1]
        t.transform.rotation.z = rot_quat[2]
        t.transform.rotation.w = rot_quat[3]
        return t
    except:
        return geometry_msgs.msg.TransformStamped()


class Data_Storer:
    def __init__(self, array = np.zeros(3), pose = geometry_msgs.msg.TransformStamped()):
        self.array = array
        self.pose = pose
        #print("hello")

class PointcloudProcess:

    #objects = None
    #poses = None
    """
    Wraps the processing of a pointcloud from an input ros topic and publishing
    to another PointCloud2 topic.

    """
    def __init__(self, points_sub_topic, 
                       image_sub_topic,
                       cam_info_topic,
                       points_pub_topic):

        self.num_steps = 0

        self.messages = deque([], 5)
        self.pointcloud_frame = None
        points_sub = message_filters.Subscriber(points_sub_topic, PointCloud2)
        image_sub = message_filters.Subscriber(image_sub_topic, Image)
        caminfo_sub = message_filters.Subscriber(cam_info_topic, CameraInfo)

        self._bridge = CvBridge()
        self.listener = tf.TransformListener()
        
        self.points_pub = rospy.Publisher(points_pub_topic, PointCloud2, queue_size=10)
        self.image_pub = rospy.Publisher('segmented_image', Image, queue_size=10)
        
        ts = message_filters.ApproximateTimeSynchronizer([points_sub, image_sub, caminfo_sub],
                                                          10, 0.1, allow_headerless=True)
        ts.registerCallback(self.callback)

    def callback(self, points_msg, image, info):
        try:
            intrinsic_matrix = get_camera_matrix(info)
            rgb_image = ros_numpy.numpify(image)
            points = ros_numpy.numpify(points_msg)
        except Exception as e:
            rospy.logerr(e)
            return
        self.num_steps += 1
        self.messages.appendleft((points, rgb_image, intrinsic_matrix))

    def publish_once_from_queue(self):
        if self.messages:
            points, image, info = self.messages.pop()
            try:
                trans, rot = self.listener.lookupTransform(
                                                       '/camera_color_optical_frame',
                                                       '/camera_depth_optical_frame',
                                                       rospy.Time(0))
                rot = tf.transformations.quaternion_matrix(rot)[:3, :3]
            except (tf.LookupException,
                    tf.ConnectivityException, 
                    tf.ExtrapolationException) as e:
                print(e)
                return [0, 0, 0]
            points = isolate_objects_of_interest(points, image, info, 
                np.array(trans), np.array(rot))
            my_data = []
            br = tf2_ros.TransformBroadcaster()
            for idx, cloud in enumerate(points):
                points_msg = numpy_to_pc2_msg(cloud)
                self.points_pub.publish(points_msg)
                temp = pca_object(cloud)
                pose = align_axes(cloud, idx)
                br.sendTransform(pose)
                my_data.append(Data_Storer(temp, pose))
            return my_data
        else:
            return [Data_Storer()]

    def getObjectType():
        return self.object

    def getPose():
        return self.pose

def main():
    CAM_INFO_TOPIC = '/camera/color/camera_info'
    RGB_IMAGE_TOPIC = '/camera/color/image_raw'
    POINTS_TOPIC = '/camera/depth/color/points'
    POINTS_PUB_TOPIC = 'segmented_points'

    rospy.init_node('realsense_listener')
    process = PointcloudProcess(POINTS_TOPIC, RGB_IMAGE_TOPIC,
                                CAM_INFO_TOPIC, POINTS_PUB_TOPIC)
    pub = rospy.Publisher('objects', Pickup, queue_size=10)
    r = rospy.Rate(1000)
    ii = 0
    arrayFlag = True
    sigma_array = [[[0]*3]*20]*10
    last_object_type = []
    buffer_idx = 0

    while not rospy.is_shutdown():
        my_data = process.publish_once_from_queue()
        poseList = []
        object_type = []
        if len(sigma_array) < len(my_data):
            sigma_array = [[[0]*3]*20]*len(my_data)
        for idx, data in enumerate(my_data):
            sigma_array[idx][ii][:] = data.array
            poseList.append(data.pose)
            pca_vals = np.nanmean(sigma_array[idx], 0)
            object_type.append(classify_object(pca_vals))
        ii = (ii + 1) % 20
        nonzeros = [item != 0 for item in object_type]
        print(object_type)

        if last_object_type != object_type:
            buffer_idx = 0
            last_object_type = object_type
        else:
            buffer_idx += 1

        if buffer_idx > 50:
            buffer_idx = 0
            try:
                pub.publish(np.array(object_type)[nonzeros], np.array(poseList)[nonzeros])
            except Exception:
                traceback.print_exc()
            r.sleep()

if __name__ == '__main__':
    main()
