#! /usr/bin/env python

import sys
import os
import pickle
import tf
import tf.transformations as tft
import math
import numpy as np
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Pose, Quaternion, Point
import rospy
import geometry_msgs.msg
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseGoal, MoveBaseActionGoal

from geometry_msgs.msg import PoseStamped
from ar_track_alvar_msgs.msg import AlvarMarkers
import fetch_api
import rospy
from robot_controllers_msgs.msg import QueryControllerStatesAction, QueryControllerStatesGoal, ControllerState
import actionlib

def wait_for_time():
    while rospy.Time().now().to_sec() == 0:
        pass

class ActionRunner(object):
    def __init__(self):
        print("create ActionRunner")
        rospy.init_node('action_runner')
        wait_for_time()
        # print("creating action client")
        # self._controller_client = actionlib.SimpleActionClient('/query_controller_states', QueryControllerStatesAction)
        # print("creating goal")
        # goal = QueryControllerStatesGoal()
        # print("creating controlelr state")
        # state = ControllerState()
        # state.name = 'arm_controller/follow_joint_trajectory'
        # state.state = ControllerState.RUNNING
        # print("updating goal")
        # goal.updates.append(state)
        # print("sending goal")
        # self._controller_client.send_goal(goal)
        # print("waiting...")
        # self._controller_client.wait_for_result()
        # print("done waiting!!")

        # get gripper
        self.gripper = fetch_api.Gripper()
        self.arm = fetch_api.Arm()
        self.reader = self.ArTagReader()
        self.markers = {}
        # get initial position of markers... it will continue updating in background
        while len(self.reader.markers) == 0:
            print("finding my markers....")
            rospy.sleep(0.1)
        for m in self.reader.markers:
            self.markers[m.id] = m

        print("I am done!")

    def go_to_pose(self, tag_id, wrist_in_marker):
        tag_id = int(tag_id)
        print(tag_id)
        print(wrist_in_marker)
        marker_pose_stamped = None
        for marker in self.markers:
            if marker == tag_id:
                print(tag_id, marker)
                marker_pose_stamped = self.markers[marker].pose
                break

        marker_pos = marker_pose_stamped.pose.position
        marker_ori = marker_pose_stamped.pose.orientation
        marker_in_base = np.dot(tft.translation_matrix([marker_pos.x, marker_pos.y, marker_pos.z]), tft.quaternion_matrix([marker_ori.x, marker_ori.y, marker_ori.z, marker_ori.w]))

        wrist_in_base = np.dot(marker_in_base, wrist_in_marker)
        pose = Pose(Point(*tft.translation_from_matrix(wrist_in_base)), Quaternion(*tft.quaternion_from_matrix(wrist_in_base)))
        pose_stamped = PoseStamped(pose=pose)
        pose_stamped.header.frame_id = "/base_link"
        pose_stamped.header.stamp = rospy.Time.now()
        self.arm.move_to_pose(pose_stamped)


    def close_gripper(self):
        self.gripper.close()

    def open_gripper(self):
        self.gripper.open()

    class ArTagReader(object):
        def __init__(self):
            self.markers = []
            rospy.Subscriber('ar_pose_marker', AlvarMarkers, self.callback)
        def callback(self, msg):
            self.markers = msg.markers
"""
Executes the action from action_saver.py

Starts the arm controller.
Uses undocumented feature of the Fetch robot. The fetch robot API exposes
    an action named /query_controller_states of type
    robot_controllers_msgs/QueryControllerStates.
Creates an action client to use this action.
"""
def main():
  #  start the arm controller
  # wtf
  ar = ActionRunner()
  
  print("Give me the program name!")
  name = raw_input(">")
  name = name.strip()
  loadfile = open('/home/team3/lab_29_actions/' + name, 'rb')
  commandlist = pickle.load(loadfile)
  for item in commandlist:
    if isinstance(item, tuple):
        print(item)
        ar.go_to_pose(*item)
    elif item == "open":
        ar.open_gripper()
    elif item == "close":
        ar.close_gripper()

  loadfile.close()

if __name__ == '__main__':
  main()



"""
mbagoal = MoveBaseActionGoal()
mbgoal = MoveBaseGoal()
mbgoal.target_pose = stampedCoPose #potential issue here
mbagoal.goal = mbgoal
#mbagoal.header =
#mbagoal.goal_id =
pub.publish(mbagoal)
"""
