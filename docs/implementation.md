---
order: 20
---

# Implementation

## Hardware
![Baxter](/assets/images/baxter.jpg)

### Rethink Robotics Baxter
Two-armed robot with parallel-jaw electric gripper attachment. (Provided in lab.)
[Datasheet](https://www.allied-automation.com/wp-content/uploads/2015/02/Baxter_datasheet_5.13.pdf)

![RealSense](/assets/images/realsense.jpg)
### Intel RealSense Camera
Capable of RBG image stream, IR point cloud mapping, and depth sensing. (Provided in lab.)
[Datasheet](https://www.intel.com/content/dam/support/us/en/documents/emerging-technologies/intel-realsense-technology/Intel-RealSense-D400-Series-Datasheet.pdf)

## Software
### Building Blocks
Our system was developed using the Python API of ROS Kinetic. We included the following open-source ROS packages: 
* `ar_track_alvar`: AR tag recognition, needed to localize RealSense to Baxter.
* `moveit`: motion planner for Baxter.
* `realsense-ros`: ROS interface for the RealSense.

Additionally, we took advantage of a variety of open-source Python libraries for data and image analysis, including numpy/scipy, matplotlib, cv2, and sklearn/skimage.

### Custom Message Types
**Pickup.msg**
```
float64[] items
geometry_msgs/TransformStamped[] transforms
```

## System Operation Process
### Camera Calibration
To operate our system, we need to first localize the RealSense relative to the Baxter. To do this, we start a tf listener node with `cv_algorithm/realsense_baxter_listener.py`, place a single AR tag in view of both the RealSense and Baxter's right wrist camera, and launch `cv_algorithm/ar_track.launch`. This file initializes two nodes that alternate the AR tag's transform between the RealSense tf tree and the Baxter tf tree. The listener then saves both the RealSense-to-AR transform and the Baxter-to-AR transform, calculates the transform between the `camera_depth_optical_frame` of the RealSense and the `base` frame of the Baxter, and broadcasts this to tf as a static transform, essentially connecting the two trees. From here, the ar_track nodes can be killed, and the AR tag is no longer necessary.

![calibration](/assets/charts/camera_calibration.PNG)

### Object Detection and Classification
Using the RealSense's color camera, we identify areas of the visible workspace that differ in color from the background. For our implementation, since we had white kitchenware on a dark brown/black background, we converted RGB images into the hue-saturation-value (HSV) color space, and placed lower-bound thresholds on saturation and value, to isolate all light-colored pixels in the image. From here, we convert the image into a binary mask, and use OpenCV's contour detection algorithms to find a list of all the contours (outlines) surrounding contiguous white segments of the image. Filtering each contour by its area (to get rid of small areas of noise that made it past thresholding), we end up with a list of major contours, each representing the outline of a single object on the table. Each major contour is used to generate a unique mask for each object. These operations are defined in `segmentation/image_segmentation.py`.

![contour](/assets/charts/contour_detection.PNG)

Each object's mask is passed into `segmentation/pointcloud_segmentation.py`, which projects the RealSense's pointcloud data into the frame of the color camera, and isolates only those points corresponding to white pixels of the mask. Each individual pointcloud, representing the points of each object, is passed back into `segmentation/main.py`, in order to classify the object.

To perform object classification, `segmentation/main.py` runs sklearn's implementation of PCA on each object's pointcloud, in three dimensions. The resulting singular values correspond to the major degrees of variance of each pointcloud:
* Cups have high variance in all three dimensions, so all of their singular values are relatively large.
* Plates have high variance in only two dimensions, so their first 2 singular values are large relative to the 3rd.
* Utensils like forks only have high variance in one dimension, so their first singular value is large relative to the 2nd and 3rd.
Because of this, we normalize the singular values to the 1st (largest) one, and set simple thresholds to classify the object into one of these three types.

Since the principal components returned by PCA are by definition orthogonal, we simply use these components as vectors defining the 3D orientation of the object (although this data is only useful for utensils, which cannot be grabbed lengthwise, along PCA1), by converting the component matrix into a normalized quaternion. Combining this with the median point in the pointcloud, which represents an approximate center of mass for the object, we create a pose representing that object's position in space, relative to the `camera_depth_optical_frame`. For each object present in the workspace, we broadcast this transform to tf for visualization in rviz, and append its object type (represented as a float from 0 to 3) and transform to a `Pickup` object (see Custom Message Types), which is then published to the `objects` topic, for use in planning and actuation.

![rviz_setup](/assets/charts/rviz_setup.PNG)

To minimize the effect of noise or motion on the published classification data, we publish the most recent Pickup only if the past 10 classification runs have returned the same number and type of objects. This allows the system to avoid publishing unless the workspace is stable and each object classification is certain.

### Planning and Actuation
![actuation_flow](/assets/charts/actuation_flow.png)

We rely on MoveIt for path planning, using a combination of the `BKPIECE` algorithm for longer maneuvers and Cartesian waypoint planning for short paths (wrapper code in `mv/src/path_planner.py`). The transformed coordinates from the RealSense, in the form of `Pickup.msg`, are utilized to navigate the Baxter gripper to an approximate location of the utensil. From there, the visual servo takes over, with the goal of correcting the coordinates for precise pickup. This was necessary because of the consistent errors of about 2-4 cm arising from the vision pipeline, which would adversely affect our object pickup reliability if not corrected.

### Visual Servo
![correction](/assets/wrist_vision/correction.png)

The visual servo, defined in `mv/src/objects/camera_control.py`, uses Baxter’s right_hand_camera to take an image, isolate the object of interest using image segmentation and contour detection, and compute the offset from the centre of the object to the gripper position in terms of pixels. This offset is then scaled to convert to actual distances and the exact coordinates of the centre of the object is obtained. Cartesian waypoints are used to direct the gripper to the computed coordinates and initiate the pickup sequence.


Our implementation supports the pickup of cups, spoons/forks and plates. Each supported object has a unique pickup sequence as defined in `mv/src/objects/<object>.py`. Details like the orientation of the gripper on pickup and object radius have been hard-coded for simplicity.

Once the object has been successfully picked up, it is placed at a particular predefined location, unique for each kind of object (see demo).

## System Overview

Our system requires a calibration stage, where an ar marker is placed on the table in the field of view of both the Realsense camera and the Baxter wrist camera. The ar_track.launch launch file is run to enable ar_marker transform publishing to TF from both camera's ar_track nodes, and the realsense_baxter_listener.py node is run to listen to these transforms. It computes the transform from the Realsense camera to the Baxter Base and continuously publishes this static transform during steady-state operation. 

Now the system is ready to classify kitchenware and sort them. The main.py function is run locally,so it starts segmenting pointclouds by color thresholding images in the HSV color space, doing contour detection to separate individual objects, and then for each object, it does PCA on the individual object to classify it. It sends a Pickup.msg message, which contains an array of the types of items and a corresponding array of the center of masses of each item via the objects topic. The main.py function is left running continuously and will update if new items are placed within the Realsense frame. From here, the obj_move.py node is run on Baxter, and it subscribes to the objects topic and receives the Pickup.msg. It attempts to find a plan to pick up the cup first, carries out the motion required to get close enough to get a good picture, and uses visual servoing with the wrist camera to achieve enough precision to pick up the cup. The obj_move.py function is left running continuously as well, so it will keep trying to pick up items as long as Pickup.msg is not empty and will update if new items are added.

![System Flow Chart](/docs/assets/images/high_level_schematic.jpg)
