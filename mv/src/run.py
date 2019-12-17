#!/usr/bin/env python
import rospy
import os

from baxter_interface import gripper as baxter_gripper
from path_planner import PathPlanner

from objects.cup import Cup
from objects.plate import Plate
from objects.spoon import Spoon


rospy.init_node('obj_mover', anonymous=True)
arm = "right"
planner = PathPlanner(arm + "_arm")
gripper = baxter_gripper.Gripper(arm)
os.system('rosrun baxter_tools tuck_arms.py -u')
gripper.calibrate()
gripper.close()

x, y = 0.5, -0.4
obj = Plate(x, y, gripper, planner)
#obj = Spoon(x, y, gripper, planner)

try:
    obj.perform_actions()
except Exception as e:
    print e
