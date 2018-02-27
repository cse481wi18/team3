#! /usr/bin/env python

import sys
import os
import pickle
from copy import deepcopy
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
from math import atan, sqrt

def wait_for_time():
    while rospy.Time().now().to_sec() == 0:
        pass

def compute_turn(pose):
    return atan(pose.position.y / pose.position.x)

def compute_dist(pose):
    return sqrt(pow(pose.position.x, 2) + pow(pose.position.y, 2))



class ActionRunner(object):
    def __init__(self):
        print("create ActionRunner")
        rospy.init_node('action_runner')
        wait_for_time()

        # get gripper
        self.gripper = fetch_api.Gripper()
        print("done with gripper")
        self.arm = fetch_api.Arm()
        print("done with arm")
        self.base = fetch_api.Base()
        print("done with base")
        self.reader = self.ArTagReader()
        self.markers = {}
        # get initial position of markers... it will continue updating in background
        reachable = False
        while not reachable:
            while len(self.reader.markers) == 0:
                print("waiting")
            m = self.reader.markers[0]
            pose_stamped = PoseStamped(pose=m.pose.pose)
            pose_stamped.header.frame_id = "/base_link"
            pose_stamped.header.stamp = rospy.Time.now()
            turn = compute_turn(deepcopy(pose_stamped.pose))
            print(turn)
            if abs(turn) > 0.07:
                self.base.turn(turn)
                print("done adjusting")
                rospy.sleep(5)
                if abs(compute_turn(m.pose.pose)) <= 0.1:
                    break
                else:
                    continue
            if abs(compute_turn(m.pose.pose)) <= 0.1:
                break

        print("checking forward...")
        reachable = False
        while not reachable:
            while len(self.reader.markers) == 0:
                print("waiting")
            m = self.reader.markers[0]
            pose_stamped = PoseStamped(pose=deepcopy(m.pose.pose))
            pose_stamped.header.frame_id = "/base_link"
            pose_stamped.header.stamp = rospy.Time.now()

            print(self.arm.compute_ik(pose_stamped))
            if self.arm.compute_ik(pose_stamped):
                print("reachable")
                reachable = True
            else:
                reachable = False
                print("marker is too far away from the robot")
                computed_forward = compute_dist(pose_stamped.pose) - 1.5
                print(computed_forward)
                max_forward = 0.05 if computed_forward < 0.7 else computed_forward
                print("I AM MOVING THIS MUCH: " + str(max_forward))
                self.base.go_forward(max_forward)
                print("move")
        # marker is now reachable
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
  
  """
  print("Give me the program name!")
  name = raw_input(">")
  name = name.strip()
  loadfile = open('/home/team3/lab_29_actions/' + name, 'rb')
  commandlist = pickle.load(loadfile)
  for item in commandlist:
    if isinstance(item, tuple):
        print(item)
        ar.go_to_pose(*item)

  loadfile.close()
  """

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
