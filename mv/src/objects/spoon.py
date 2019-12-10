#!/usr/bin/env python
import rospy
from camera_control import get_correction

class Spoon(object):

    def __init__(self, x, y, gripper, planner, orientation=[0.0, 1.0, 0.0, 0.0]):
        self.coord_x, self.coord_y = x, y
        self.gripper, self.planner = gripper, planner
        self.hover_z, self.pickup_z = -0.24, -0.355
        self.orient = orientation

        self.final_x, self.final_y, self.final_z = x, y, 0.0 # SET THIS ACCORDINGLY

    def perform_actions(self):

        request1 = self.planner.construct_plan([self.coord_x + 0.05, self.coord_y, self.hover_z], self.orient)
        if not self.planner.execute_plan(request1):
            raise Exception("Execution failed")
        self.gripper.open()
        rospy.sleep(1.0)

        corr_x, corr_y = get_correction()
        print(corr_x, corr_y)
        self.coord_x -= (corr_x - 0.05)
        self.coord_y -= corr_y

        request1 = self.planner.construct_plan([self.coord_x, self.coord_y, self.hover_z], self.orient)
        if not self.planner.execute_plan(request1):
            raise Exception("Execution failed")

        request2 = self.planner.waypoint_plan([self.coord_x, self.coord_y, self.pickup_z], self.orient)
        if not self.planner.execute_plan(request2):
            raise Exception("Execution failed")
        rospy.sleep(2.0)
        self.gripper.set_velocity(10.0)
        self.gripper.command_position(5.0)
        self.gripper.set_velocity(50.0) # back to default
        rospy.sleep(1.0)

        request3 = self.planner.construct_plan([self.final_x, self.final_y, self.final_z], self.orient)
        if not self.planner.execute_plan(request3):
            raise Exception("Execution failed")
        rospy.sleep(1.0)
        self.gripper.open()
        rospy.sleep(1.0)
