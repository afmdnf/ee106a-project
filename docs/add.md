---
order: 60
---

# Additional Materials

## Script for Object Pickup (`mv/src/obj_move.py`)

```python
from baxter_interface import gripper as baxter_gripper
from path_planner import PathPlanner

from objects.cup import Cup
from objects.plate import Plate
from objects.spoon import Spoon

from mv.msg import Pickup

os.system('rosrun baxter_tools camera_control.py -o right_hand_camera -r 1280x800')
rospy.init_node('obj_mover', anonymous=True)
arm = "right"
planner = PathPlanner(arm + "_arm")
gripper = baxter_gripper.Gripper(arm)
gripper.calibrate()

pro = None

def move(msg):
    items, transforms = msg.items, msg.transforms
    if not transforms or len(items) != len(transforms):
        print("[ERROR]: Invalid message received")
        return
    print("Received command for %d items" % len(items))

    # Pick up objects starting from cup and then rightmost (y) + closest (x)
    processed = sorted(zip(items, transforms), key=lambda x:(-x[0], x[1].transform.translation.y, x[1].transform.translation.x))
    for item, tr in processed:
        obj_id = int(item)
        if obj_id not in [1, 2, 3]:
            print("[ERROR]: Invalid object %d at index %d" % (obj_id, i))
            continue

        x, y = tr.transform.translation.x, tr.transform.translation.y
        obj = None
        if obj_id == 1:
            obj = Plate(x, y, gripper, planner)
        if obj_id == 2:
            orient = [tr.transform.rotation.x, tr.transform.rotation.y, tr.transform.rotation.z, tr.transform.rotation.w]
            obj = Spoon(x, y, gripper, planner)
        elif obj_id == 3:
            obj = Cup(x, y, gripper, planner)

        if obj:
            tuck()
            gripper.close()

            print("Planning for:", obj_id, x, y)
            try:
                obj.perform_actions()
            except Exception as e:
                print e
        else:
            print("[ERROR]: Invalid object %d at index %d" % (obj_id, i))

def tuck():
    global pro
    if pro:
        os.killpg(os.getpgid(pro.pid), signal.SIGTERM)
    os.system('rosrun baxter_tools tuck_arms.py -u')
    pro = subprocess.Popen("rosrun baxter_interface joint_trajectory_action_server.py", stdout=subprocess.PIPE, shell=True, preexec_fn=os.setsid)
    rospy.sleep(1.0)


if __name__ == '__main__':
    rospy.Subscriber("objects", Pickup, move, queue_size=1)
    rospy.spin()
```

## Script for Computer Vision (`segmentation/src/main.py`)
```python
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from mv.msg import Pickup, StringArray
from cv_bridge import CvBridge
from sklearn.decomposition import PCA
from image_segmentation import segment_image, show_image
from pointcloud_segmentation import segment_pointcloud
from sensor_msgs import point_cloud2


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
        return 1 # plate
    elif (sigma[1] < 0.7 and sigma[2] < 0.3):
        return 2 # spoon
    else:
        return 3 # cup
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

class PointcloudProcess:
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
                try:
                    pose_from_base = self.listener.lookupTransform('base', pose.child_frame_id, rospy.Time(0))
                    final_pose = geometry_msgs.msg.TransformStamped()
                    final_pose.transform.translation.x = pose_from_base[0][0]
                    final_pose.transform.translation.y = pose_from_base[0][1]
                    final_pose.transform.translation.z = pose_from_base[0][2]

                    final_pose.transform.rotation.x = pose_from_base[1][0]
                    final_pose.transform.rotation.y = pose_from_base[1][1]
                    final_pose.transform.rotation.z = pose_from_base[1][2]
                    final_pose.transform.rotation.w = pose_from_base[1][3]
                    final_pose.header.stamp = rospy.Time.now()
                    final_pose.header.frame_id = "base"
                    final_pose.child_frame_id = pose.child_frame_id
                    my_data.append(Data_Storer(temp, final_pose))
                except Exception as e:
                    print(e)
                    return [Data_Storer()]
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
    pub = rospy.Publisher('objects', Pickup, queue_size=1)
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

        if buffer_idx > 10:
            buffer_idx = 0
            try:
                pub.publish(np.array(object_type)[nonzeros], np.array(poseList)[nonzeros])
            except Exception:
                traceback.print_exc()
            r.sleep()
```


## Our Code

The rest of our code can be found on our [GitHub](https://github.com/afmdnf/ee106a-project). 

Some software resources that we used:  
* [MoveIt](https://github.com/ros-planning/moveit)
* [AR_Track_Alvar](https://github.com/ros-perception/ar_track_alvar.git)
* [RealSense ROS](https://github.com/IntelRealSense/realsense-ros)


## Hardware Data Sheets

* [Rethink Robotics Baxter](https://www.allied-automation.com/wp-content/uploads/2015/02/Baxter_datasheet_5.13.pdf)
* [Intel RealSense D435](https://www.intel.com/content/dam/support/us/en/documents/emerging-technologies/intel-realsense-technology/Intel-RealSense-D400-Series-Datasheet.pdf)

## Videos

<iframe width="560" height="315" src="https://www.youtube.com/embed/yT-osnECr04" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
<iframe width="560" height="315" src="https://www.youtube.com/embed/wcvgII80D3w" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
<iframe width="560" height="315" src="https://www.youtube.com/embed/kIB_kAqCUNY?start=18" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

## Miscellaneous pictures
* Example Baxter wrist camera image, with cup in sight
![wrist_cam](/assets/wrist_vision/hand.png)

* Extra setup photos
![extra_1](/assets/setup_images/setup_1b.jpg)
![extra_2](/assets/setup_images/setup_2b.jpg)

* Extra RViz screencaps
![rviz_a](/assets/rviz/rviz1.png)
![rviz_b](/assets/rviz/rviz3.png)