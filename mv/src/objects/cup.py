#!/usr/bin/env python
import rospy
from camera_control import get_correction

class Cup(object):

    def __init__(self, x, y, gripper, planner):
        self.coord_x, self.coord_y = x, y
        self.gripper, self.planner = gripper, planner

        self.radius = 0.03
        self.hover_z, self.pickup_z = -0.12, -0.296 # OLD TABLE
        #self.hover_z, self.pickup_z = 0.011, -0.187
        self.pickup_y = self.coord_y + self.radius
        self.orient = [0.0, 1.0, 0.0, 0.0]

        self.final_x, self.final_y, self.final_z = 0.46, -0.96, 0.0#-0.08, -0.9, 0.0#x, y, 0.0 # SET THIS ACCORDINGLY

    def perform_actions(self):
        camera_move_ahead = 0.05
        request1 = self.planner.construct_plan([self.coord_x + camera_move_ahead, self.coord_y, self.hover_z], self.orient)
        self.planner.execute_plan(request1)

        corr_x, corr_y = get_correction("cup")
        self.coord_x -= (corr_x - camera_move_ahead)
        self.coord_y -= corr_y
        self.pickup_y = self.coord_y + self.radius

        request1 = self.planner.construct_plan([self.coord_x, self.coord_y, self.hover_z], self.orient)
        self.planner.execute_plan(request1)
        #rospy.sleep(1.0)

        request2 = self.planner.waypoint_plan([self.coord_x, self.pickup_y, self.hover_z], self.orient)
        self.planner.execute_plan(request2)
        self.gripper.open()
        rospy.sleep(1.0)

        request3 = self.planner.waypoint_plan([self.coord_x, self.pickup_y, self.pickup_z], self.orient)
        self.planner.execute_plan(request3)
        self.gripper.close()
        rospy.sleep(1.0)

        request4 = self.planner.waypoint_plan([self.coord_x, self.pickup_y, self.hover_z], self.orient)
        self.planner.execute_plan(request4)
        #rospy.sleep(1.0)

        request5 = self.planner.construct_plan([self.final_x, self.final_y, self.final_z], self.orient)
        self.planner.execute_plan(request5)
        self.gripper.open()
        rospy.sleep(1.0)
