#!/usr/bin/env python
import rospy
import numpy as np

class Plate(object):

    def __init__(self, x, y, gripper, planner):
        self.gripper, self.planner = gripper, planner
  
        offset, gripper_correction = 0.025, 0.04
        self.radius = 0.05

        self.coord_x, self.coord_y = x + gripper_correction, y + gripper_correction
        self.hover_z, self.pickup_z = -0.22, -0.309
        self.hover_x = x - (self.radius + offset) / np.sqrt(2)
        self.pickup_x = x - self.radius / np.sqrt(2)
        self.hover_y = y - (self.radius + offset) / np.sqrt(2)
        self.pickup_y = y - self.radius / np.sqrt(2)

        self.orient = np.array([0.22, 0.781, 0.555, 0.179])
        self.orient /= np.linalg.norm(self.orient)

        self.final_x, self.final_y, self.final_z = self.coord_x, self.coord_y, 0.0 # SET THIS ACCORDINGLY

    def perform_actions(self):

        request1 = self.planner.construct_plan([self.coord_x, self.coord_y, self.hover_z], self.orient)
        if not self.planner.execute_plan(request1):
            raise Exception("Execution failed")
        rospy.sleep(1.0)

        request2 = self.planner.waypoint_plan([self.hover_x, self.hover_y, self.hover_z], self.orient)
        if not self.planner.execute_plan(request2):
            raise Exception("Execution failed")
        rospy.sleep(1.0)

        request3 = self.planner.waypoint_plan([self.hover_x, self.hover_y, self.pickup_z], self.orient)
        if not self.planner.execute_plan(request3):
            raise Exception("Execution failed")
        rospy.sleep(1.0)
        self.gripper.open()
        rospy.sleep(1.0)

        request4 = self.planner.waypoint_plan([self.pickup_x, self.pickup_y, self.pickup_z], self.orient)
        if not self.planner.execute_plan(request4):
            raise Exception("Execution failed")
        rospy.sleep(1.0)
        self.gripper.close()
        rospy.sleep(1.0)

        request5 = self.planner.construct_plan([self.final_x, self.final_y, self.final_z], self.orient)
        if not self.planner.execute_plan(request5):
            raise Exception("Execution failed")
        rospy.sleep(1.0)
        self.gripper.open()
        rospy.sleep(1.0)
