#!/usr/bin/env python
import rospy
from camera_control import get_correction
from baxter_core_msgs.msg import EndEffectorState

class Spoon(object):

    def __init__(self, x, y, gripper, planner, orientation=[0.0, 1.0, 0.0, 0.0]):
        self.gripper_update = rospy.Subscriber("/robot/end_effector/right_gripper/state", EndEffectorState, self.record_state)

        self.coord_x, self.coord_y = x, y
        self.gripper, self.planner = gripper, planner
        self.hover_z, self.pickup_z = -0.24, -0.355
        self.orient = orientation
        self.gripper_state = 100.0

        self.final_x, self.final_y, self.final_z = x, y, 0.0 # SET THIS ACCORDINGLY

    def perform_actions(self):
        camera_move_ahead = 0.05
        request1 = self.planner.construct_plan([self.coord_x + camera_move_ahead, self.coord_y, self.hover_z], self.orient)
        self.execute(request1)
        # if not self.planner.execute_plan(request1):
        #     raise Exception("Execution failed 1")
        # while (not self.gripper.open()):
        #     continue
        self.gripper.open()
        rospy.sleep(1.0)

        corr_x, corr_y = get_correction("spoon")
        self.coord_x -= (corr_x - camera_move_ahead)
        self.coord_y -= corr_y

        request1 = self.planner.construct_plan([self.coord_x, self.coord_y, self.hover_z], self.orient)
        self.execute(request1)
        # if not self.planner.execute_plan(request1):
        #     raise Exception("Execution failed 2")

        request2 = self.planner.waypoint_plan([self.coord_x, self.coord_y, self.pickup_z], self.orient)
        self.execute(request2)
        # if not self.planner.execute_plan(request2):
        #     raise Exception("Execution failed 3")
        rospy.sleep(1.0)
        self.gripper.set_velocity(10.0)
        while (self.gripper_state > 10): # self.gripper.command_position(5.0)
            self.gripper.close()
        self.gripper.set_velocity(50.0) # back to default
        rospy.sleep(1.0)

        request3 = self.planner.construct_plan([self.final_x, self.final_y, self.final_z], self.orient)
        self.execute(request3)
        # if not self.planner.execute_plan(request3):
        #     raise Exception("Execution failed 4")
        rospy.sleep(1.0)
        self.gripper.open()
        # while (not self.gripper.open()):
        #     continue
        rospy.sleep(1.0)

    def execute(self, request):
        while(not self.planner.execute_plan(request)):
            continue

    def record_state(self, state):
        self.gripper_state = state.position