#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
import numpy as np
import geometry_msgs
from geometry_msgs.msg import Vector3, Quaternion
import tf
import tf2_ros
import tf_conversions
import roscpp

#Define the callback method which is called whenever this node receives a 
#message on its subscribed topic. The received message is passed as the 
#first argument to callback().
def publishpose(pose):
	br = tf2_ros.StaticTransformBroadcaster()
	br.sendTransform(pose)
	print(pose)
	return
#Define the method which contains the node's main functionality
#def listener():

	#Run this program as a new node in the ROS computation graph
	#called /listener_<id>, where <id> is a randomly generated numeric
	#string. This randomly generated name means we can start multiple
	#copies of this node without having multiple nodes with the same
	#name, which ROS doesn't allow.
	

#Python's syntax for a main() method
if __name__ == '__main__':
	rospy.init_node('listener', anonymous=True)

	#Create a new instance of the rospy.Subscriber object which we can 
	#use to receive messages of type std_msgs/String from the topic /chatter_talk.
	#Whenever a new message is received, the method callback() will be called
	#with the received message as its first argument.
	tfBuffer = tf2_ros.Buffer()
	tfListener = tf2_ros.TransformListener(tfBuffer)
	# Create a timer object that will sleep long enough to result in
	# a 10Hz publishing rate
	#r = rospy.Rate(10) # 10hz
	# Loop until the node is killed with Ctrl-C
	realsense_to_ar = None
	while realsense_to_ar == None:
		try:
			realsense_to_ar = tfBuffer.lookup_transform("ar_marker_5", "camera_depth_optical_frame", rospy.Time())
			#print(dir(realsense_to_ar))
			print("Got Realsense Position!")
		except:
			pass


	baxter_to_ar = None
	while baxter_to_ar == None:
		try:
			baxter_to_ar = tfBuffer.lookup_transform("ar_marker_5", "base", rospy.Time())
			print("Got the Baxter position!")
		except:
			pass

	trans_vec_1 = np.array([realsense_to_ar.transform.translation.x,realsense_to_ar.transform.translation.y,realsense_to_ar.transform.translation.z,1])
	trans_vec_2 = np.array([baxter_to_ar.transform.translation.x,baxter_to_ar.transform.translation.y,baxter_to_ar.transform.translation.z,1])
	quat_vec_1 = np.array([realsense_to_ar.transform.rotation.x,realsense_to_ar.transform.rotation.y,realsense_to_ar.transform.rotation.z,realsense_to_ar.transform.rotation.w])
	quat_vec_2 = np.array([baxter_to_ar.transform.rotation.x,baxter_to_ar.transform.rotation.y,baxter_to_ar.transform.rotation.z,baxter_to_ar.transform.rotation.w])
	quat_mat_1 = tf.transformations.quaternion_matrix(quat_vec_1)
	quat_mat_2 = tf.transformations.quaternion_matrix(quat_vec_2)
	quat_mat_1[:,3] = trans_vec_1
	quat_mat_2[:,3] = trans_vec_2
	full_transform = np.dot(np.linalg.inv(quat_mat_1), quat_mat_2)
	final_transform = geometry_msgs.msg.Transform()

	final_trans_vec = np.copy(full_transform[0:3,3])
	full_transform[0:3,3] = 0
	final_quat_vec = tf.transformations.quaternion_from_matrix(full_transform)
	t = geometry_msgs.msg.TransformStamped()
	t.header.stamp = rospy.Time.now()
	t.header.frame_id = "camera_depth_optical_frame"
	t.child_frame_id = "base"
	#return final_transform
	final_transform.translation.x = final_trans_vec[0]
	final_transform.translation.y = final_trans_vec[1]
	final_transform.translation.z = final_trans_vec[2]
	final_transform.rotation.x = final_quat_vec[0]
	final_transform.rotation.y = final_quat_vec[1]
	final_transform.rotation.z = final_quat_vec[2]
	final_transform.rotation.w = final_quat_vec[3]
	t.transform = final_transform
	# Create a timer object that will sleep long enough to result in
	# a 10Hz publishing rate
	#r = rospy.Rate(10) # 10hz
	# Loop until the node is killed with Ctrl-C
	while not rospy.is_shutdown():
		try:
			publishpose(t)
		except rospy.ROSInterruptException as e:
			print(e)
