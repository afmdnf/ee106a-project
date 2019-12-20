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
* ar_track_alvar: AR tag recognition, needed to localize RealSense to Baxter.
* moveit: motion planner for Baxter.
* realsense-ros: ROS interface for the RealSense.
Additionally, we took advantage of a variety of open-source Python libraries for data and image analysis, including numpy/scipy, matplotlib, cv2, and sklearn/skimage.

### Software We Wrote
*TODO: Finish these sections, add flow diagrams*

#### Custom Message Types
**Pickup.msg**
```
float64[] items
geometry_msgs/TransformStamped[] transforms
```

#### Actuation
![actuation_flow](/assets/charts/actuation_flow.png)

We rely on MoveIt for path planning, using a combination of the `BKPIECE` algorithm for longer maneuvers and Cartesian waypoint planning for short paths (wrapper code in `mv/src/path_planner.py`). The transformed coordinates from the RealSense, in the form of `Pickup.msg`, are utilized to navigate the Baxter gripper to an approximate location of the utensil. From there, the visual servo takes over, with the goal of correcting the coordinates for precise pickup. This was necessary because of the consistent errors of about 2-4 cm arising from the vision pipeline, which would adversely affect our object pickup reliability if not corrected.

**Visual Servo**
![correction](/assets/wrist_vision/correction.png)
The visual servo, defined in `mv/src/objects/camera_control.py`, uses Baxter’s right_hand_camera to take an image, isolate the object of interest using image segmentation and contour detection, and compute the offset from the centre of the object to the gripper position in terms of pixels. This offset is then scaled to convert to actual distances and the exact coordinates of the centre of the object is obtained. Cartesian waypoints are used to direct the gripper to the computed coordinates and initiate the pickup sequence.


Our implementation supports the pickup of cups, spoons/forks and plates. Each supported object has a unique pickup sequence as defined in `mv/src/objects/<object>.py`. Details like the orientation of the gripper on pickup and object radius have been hard-coded for simplicity.

Once the object has been successfully picked up, it is placed at a particular predefined location, unique for each kind of object (see demo).

XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX

To actually drive the robot, we wrote a python script called RoverGoSmooth.py. After starting a roscore and launching the aforementioned packages necessary to run the kinect and AR tracking, RoverGoSmooth.py can be ran in a new terminal to activate the robot. A script will prompt the user for the desired AR-tag destination and then the robot will drive to its target and attempt to grab the object located near that target. RoverGoSmooth.py works by using a tf transform listener to repeatedly look up the most recent transformation to the target AR-tag The robot then uses this information to make a decision about where to drive next, and drives in that direction for a short burst. When the robot is finished driving for that particular step, it pauses for a short time period to allow ar_track_alvar to update its information about the AR-tags. If the tag is not visible when the robot is started, the robot will periodically make small turns in a circle and essentially “search” for the AR-tag. Once the tag is visible, the translation data from the tf transform is used to compute the distance to the tag, as well as an angle that represents how far off center the tag is within the robots current field of view.

Transforms are computed relative the the Kinect frame, camera_link. This frame is convenient to use because the x-axis of the frame always points in the same direction as the camera’s lense. The distance is computed via a simple 2 dimensional distance formula (z direction is not relevant for driving in the x-y plane). The angle is simple the angle offset of the target AR-tag relative to the camera_link frames x-axis, where an angle of zero tells the robot that the target is straight ahead. At each driving step, the robot computes this distance and angle and makes a decision. First, the robot checks if the tag is within a given angle tolerance, if it is not, it uses the sign of the computed angle to decide to turn left or right. If the angle is within tolerance, the robot then checks to see if the robot is within a given distance tolerance. If it is not, the robot drives forward. Otherwise the robot will attempt a grab via another python script we wrote, grabBlock.py.

![TF Calculation Diagram](/assets/robot_images/rover_tf_calculation_diagram.jpg)

When the robot is actually moving, it needs to either drive straight or turn accurately. To accomplish this, we implemented a very simple proportional controller that uses photo interrupters to compute approximately how far each wheel has turned. The controller then attempts to adjust the input of one wheel while holding the other input constant so that the two wheels always travel an equal distance so that the robot can drive straight. Note that because the robot drives in short bursts, this controller is mostly a safeguard, as the most important aspect for getting our robot to drive straight was calibrating the initial inputs to the wheels so that it at least starts off driving straight initially. Calibrating these inputs was done simply by trial and error.

We also included some logic that prompts the robot to search for the target again if it loses sight of the tag while driving. The most recent previous known location of the tag is used to attempt to make an accurate choice about which direction to turn in this case. This python script could have been turned into an actual ROS node, but given how it functions we felt that this was unnecessary and overly complicated, so we decided to just keep it as a simple python script. The other script that we wrote, grabBlock.py applies inputs to the robot arm that causes it to reach out in front of the robot and attempt a grab. This code for this script is simply a sequence of hard coded inputs to the arm that causes the robot to attempt to grab whatever is currently in front of it.

![Rover Flow Chart](/assets/robot_images/rover_flow_chart.jpg)

### System Overview

Overall, our system works by continuously using the Kinect to collect image data. Ar_track_alvar then takes this data and computes which AR tags are visible. Tf then computes the transformations to each of the tags. While this is all happening, RoverGoSmooth.py can be ran with a particular AR tag selection. RoverGoSmooth.py then periodically looks up the transformation to the desired AR tag and makes a decision about which direction to move in. Once the robot is within the desired distance and angle tolerances relative to its desired AR tag, the attempts to grab the object at the AR tag via grabBlock.py

![System Flow Chart](/assets/robot_images/system_flow_chart.jpg)
