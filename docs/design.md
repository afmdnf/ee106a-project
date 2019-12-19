---
order: 10
---

# Design

## Design Criteria

Our group's primary design criterion was to be able to reliably pick up 3 different object types — plates, cups and forks — and move them to a desired location, effectively sorting all objects in the workspace. This naturally required enforcement of a secondary criterion: namely, the ability to reliably detect both the number of objects in the workspace, as well each object's type and position, with high confidence. Finally, we wanted our system to be able to perform this detection continuously, and update Baxter with the state of the workspace in real time.

## Our Design

The final design of our system uses images from a RealSense's standard camera to isolate all objects of interest from a colorless background. Binary image masks for each object are then used to segment a pointcloud obtained from the RealSense's depth camera. Each object's pointcloud is analyzed using 3-dimensional principal component analysis (PCA) to determine its type and orientation. This information is then continuously published to a ROS topic, where an action-planning node using the MoveIt interface can instruct Baxter to move a parallel-jaw gripper to the object's location, pick it up, and set it in a designated location. During pickup, rudimentary visual servoing using image data from the Baxter wrist camera is used to fine-tune the position of the gripper.

## Design Choices and Tradeoffs

* When making our design, we chose to use the RealSense camera and the Baxter model, since these materials were readily available in lab and we were initially planning to perform two-handed manipulation, which would not be possible with the Sawyers provided in lab. Using an external depth camera made it much easier to provide an overview of the whole workspace at all times than using purely the Baxter cameras/depth sensors. However, this required us to spend a significant amount of time developing a calibration system for localizing the RealSense with respect to the Baxter, using a common AR marker.
* To keep our robot simple, we chose to only classify three different object types: cups, plates and forks. These objects were chosen because they are very distinct in their point distributions — cups are primarily distributed in 3 dimensions, plates in 2, and forks/other utensils in only 1 — making it easier to classify each object quickly, using simple thresholds on normalized singular values. However, this severely restricted the possible objects we could use to test the system, meaning the techniques we used would not scale well with additional object types, such as bowls.
* Due to time constraints, moving the Baxter end-effector to each object's position and picking the object up was done primarily with open-loop control and a series of hard-coded movements. To adjust for errors in the initial location of the gripper, we used only a single image from from the Baxter wrist camera to make small corrections. While this solution worked fairly well, it certainly isn't optimal for making the system more flexible and usable in a variety of different conditions, where closed-loop control using real-time data from the wrist cameras and Baxter's built-in depth sensors would be much more effective.
* We wanted our device to work while making no assumptions about each object's color, beyond the fact that it must be a different color than the background table. For example, we couldn't assume that all plates were red, which would allow the use of simple RGB thresholding to identify each object type. While this allows our system to adapt more easily to objects with a wide range of colors and shapes, it required us to use more complicated computer vision techniques to isolate and identify each object.

## Design Choices' Impact

**TODO: This section** The incorporation of AR-tags in our design could in theory be utilized in a real-world application, however this would not be ideal, as it would be much more robust and efficient to have the robot recognize the actual target objects. Having the robot drive in discrete steps was a vital design choice that greatly improved the accuracy of our robot. Having the robot drive continuously would result in a more polished looking final product, but accuracy is far more important for any real-world application. Finally, utilizing the robot arm in a more sophisticated way would improve the accuracy of the robot and the robustness of the overall project in a real-world application, but for our design criteria a hardcoded arm was sufficient.


