#!/usr/bin/env python
import rospy
import os
import signal
import subprocess

from baxter_interface import gripper as baxter_gripper
#from moveit_msgs.msg import OrientationConstraint
from path_planner import PathPlanner

from objects.cup import Cup
from objects.plate import Plate
from objects.spoon import Spoon

from mv.msg import Pickup


os.system('rosrun baxter_tools camera_control.py -o left_hand_camera -r 1280x800')
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
    #os.system('rosrun baxter_tools tuck_arms.py -u')

    # Pick up objects starting from cup and then rightmost (y) + closest (x)
    processed = sorted(zip(items, transforms), key=lambda x:(-x[0], x[1].transform.translation.y, x[1].transform.translation.x))
    for item, tr in processed:
        obj_id = int(item)
        if obj_id not in [1, 2, 3]:
            print("[ERROR]: Invalid object %d at index %d" % (obj_id, i))
            continue

        x, y = tr.transform.translation.x, tr.transform.translation.y
        # TODO: Check for valid x,y here

        #x, y = 0.72, -0.257 # POSITION 1
        #x, y = 0.715, -0.109 # POSITION 2

        obj = None
        if obj_id == 1:
            if y < -0.16:
                obj = Plate(x, y, gripper, planner, False)
            else:
                obj = Plate(x, y, gripper, planner, True)
        if obj_id == 2:
            orient = [tr.transform.rotation.x, tr.transform.rotation.y, tr.transform.rotation.z, tr.transform.rotation.w]
            print("SPOON ORIENTATION:", orient)
            obj = Spoon(x, y, gripper, planner)#, orient) # optionally, provide spoon orientation?
        elif obj_id == 3:
            obj = Cup(x, y, gripper, planner)

        if obj:
            tuck()
            gripper.close()

            print("Planning for:", obj_id, x, y)
            #raw_input('Press [ Enter ]: ')
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


if __name__ == '__main__':
    rospy.Subscriber("objects", Pickup, move, queue_size=1)
    rospy.spin()



# SETUP:
# ./baxter.sh archytas.local
# rosrun baxter_tools enable_robot.py -e
# rosrun baxter_interface joint_trajectory_action_server.py
# roslaunch baxter_moveit_config demo_baxter.launch right_electric_gripper:=true left_electric_gripper:=true
# rosrun mv obj_move.py

# DOCS:
# https://ros-planning.github.io/moveit_tutorials/doc/move_group_python_interface/move_group_python_interface_tutorial.html
# http://docs.ros.org/kinetic/api/moveit_commander/html/classmoveit__commander_1_1move__group_1_1MoveGroupCommander.html


# obstacle = PoseStamped()
# obstacle.header.frame_id = "base";
# obstacle.pose.position.x, obstacle.pose.position.y, obstacle.pose.position.z = 0.5, 0.0, 0.0
# obstacle.pose.orientation.x, obstacle.pose.orientation.y, obstacle.pose.orientation.z = 0.0, 0.0, 0.0
# obstacle.pose.orientation.w = 1.0
# planner.add_box_obstacle(np.array([0.4, 1.2, 0.1]), "box", obstacle)

# orien_const = OrientationConstraint()
# orien_const.link_name = "right_gripper"
# orien_const.header.frame_id = "base"
# orien_const.orientation.y = 1.0
# orien_const.absolute_x_axis_tolerance = 0.5
# orien_const.absolute_y_axis_tolerance = 0.5
# orien_const.absolute_z_axis_tolerance = 0.5
# orien_const.weight = 1.0
