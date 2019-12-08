#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
import numpy as np
import geometry_msgs
import tf2_ros
import tf_conversions

#Define the callback method which is called whenever this node receives a 
#message on its subscribed topic. The received message is passed as the 
#first argument to callback().
def callback(message):

    #Print the contents of the message to the console
    gen = point_cloud2.read_points(message, skip_nans=True, field_names=("x", "y", "z"))
    averaged_data = np.zeros((message.width,3))
    for i, p in enumerate(gen):
        r = np.array([p[0],p[1],p[2]])
        averaged_data[i,:] = p

    position = np.median(averaged_data,axis=0)
    

    #br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "camera_depth_optical_frame"
    t.child_frame_id = "object_center_of_mass"
    t.transform.translation.x = position[0]
    t.transform.translation.y = position[1]
    t.transform.translation.z = position[2]
    #q = tf_conversions.transformations.quaternion_from_euler(0, 0, 0)
    t.transform.rotation.x = 0
    t.transform.rotation.y = 0
    t.transform.rotation.z = 0
    t.transform.rotation.w = 1
    #print(t)
    #br.sendTransform(t)
    return t

    # com_publisher = rospy.Publisher('tf', TFMessage, queue_size=10)
    # com = TFMessage()
    # com.transforms = TransformStamped()
    # com.transforms.
    # com_publisher.publish(user_input, rospy.get_time())


        # k = 0
        # for ii in range(message.width):
        #     if state[ii] == k:
        #         averaged_data[ii,:] = r
        #         state[ii] += 1
        #         if ii == message.width - 1:
        #             k += 1
        #         break
        #     else:
        #         continue
        # print(averaged_data[22:28,:])
        # print(k) #* message.height)

    #print(gen.data)#print("Message: %s, Sent at: %s, Received at: %s" % (message.header, message.data, message.fields))

#Define the method which contains the node's main functionality
def listener():

    #Run this program as a new node in the ROS computation graph
    #called /listener_<id>, where <id> is a randomly generated numeric
    #string. This randomly generated name means we can start multiple
    #copies of this node without having multiple nodes with the same
    #name, which ROS doesn't allow.
    rospy.init_node('listener', anonymous=True)

    #Create a new instance of the rospy.Subscriber object which we can 
    #use to receive messages of type std_msgs/String from the topic /chatter_talk.
    #Whenever a new message is received, the method callback() will be called
    #with the received message as its first argument.
    rospy.Subscriber("segmented_points", PointCloud2, callback)

    


    #Wait for messages to arrive on the subscribed topics, and exit the node
    #when it is killed with Ctrl+C
    rospy.spin()


#Python's syntax for a main() method
if __name__ == '__main__':
    listener()
