#!/usr/bin/env python
import rospy
from mv.msg import Pickup
from main import PointcloudProcess
def talker(objects, transforms):
    rospy.init_node('object_listener', anonymous=True)

    pub = rospy.Publisher('objects', Pickup, queue_size=10)
    r = rospy.Rate(10)
    msg = Pickup()
    msg.objects, msg.transforms = objects, transforms
    pub.publish(msg, rospy.get_time())

if __name__ == '__main__':
	while True:
		objects = PointcloudProcess.objects
		poses = PointcloudProcess.poses
		print(objects)
		talker(objects, poses)
