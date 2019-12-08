#!/usr/bin/env python
import rospy

class Cup(object):

    def __init__(self, x, y, gripper, planner):
        self.coord_x, self.coord_y = x, y
        self.gripper, self.planner = gripper, planner

        self.radius = 0.03
        self.hover_z, self.pickup_z = -0.22, -0.296
        self.pickup_y = self.coord_y + self.radius
        self.orient = [0.0, 1.0, 0.0, 0.0]

        self.final_x, self.final_y, self.final_z = x, y, 0.0 # SET THIS ACCORDINGLY

    def perform_actions(self):

        request1 = self.planner.construct_plan([self.coord_x, self.coord_y, self.hover_z], self.orient)
        if not self.planner.execute_plan(request1):
            raise Exception("Execution failed")
        rospy.sleep(1.0)

        request2 = self.planner.waypoint_plan([self.coord_x, self.pickup_y, self.hover_z], self.orient)
        if not self.planner.execute_plan(request2):
            raise Exception("Execution failed")
        self.gripper.open()
        rospy.sleep(1.0)

        request3 = self.planner.waypoint_plan([self.coord_x, self.pickup_y, self.pickup_z], self.orient)
        if not self.planner.execute_plan(request3):
            raise Exception("Execution failed")
        self.gripper.close()
        rospy.sleep(1.0)

        request4 = self.planner.construct_plan([self.final_x, self.final_y, self.final_z], self.orient)
        if not self.planner.execute_plan(request4):
            raise Exception("Execution failed")
        rospy.sleep(1.0)
        self.gripper.open()
        rospy.sleep(1.0)
