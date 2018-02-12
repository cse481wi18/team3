#! /usr/bin/env python

import sys
import os
import pickle
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped, Point, Quaternion
import rospy
import geometry_msgs.msg
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseGoal, MoveBaseActionGoal

from ar_track_alvar_msgs.msg import AlvarMarkers
import fetch_api

import tf
import tf.transformations as tft
import numpy as np

import actionlib
import robot_controllers_msgs.msg
from robot_controllers_msgs.msg import QueryControllerStatesAction, QueryControllerStatesGoal, ControllerState

def wait_for_time():
    while rospy.Time().now().to_sec() == 0:
        pass

def lookup_transform(a, b):
    listener = tf.TransformListener()
    rospy.sleep(0.1)
    pose = None
    while pose is None:
        try:
            pose = listener.lookupTransform(a, b, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass
    return Pose(Point(*pose[0]), Quaternion(*pose[1]))

class actionSaver(object):
  def __init__(self, action_name):
      #not sure if we need this
    print "create ActionSaver"
    rospy.init_node('action_saver')
    wait_for_time()

    """
    # SHOULD BE RAN ON THE REAL ROBOT
    # stop the arm controller (relax)
    controller_client = actionlib.SimpleActionClient('/query_controller_states', QueryControllerStatesAction)
    goal = QueryControllerStatesGoal()
    state = ControllerState()
    state.name = 'arm_controller/follow_joint_trajectory'
    state.state = ControllerState.STOPPED
    goal.updates.append(state)
    controller_client.send_goal(goal)
    controller_client.wait_for_result()
    """
    # list of actions
    self.list = []

    self.gripper = fetch_api.Gripper()
    self.arm = fetch_api.Arm()
    self.name = action_name
    # get the ArTagReader and attempt to get all ArTags found
    self.reader = self.ArTagReader()
    sub = rospy.Subscriber('ar_pose_marker', AlvarMarkers, self.reader.callback) # Subscribe to AR tag poses
    # while AR tags cannot be found
    while len(self.reader.markers) == 0:
      print "finding markers...."
      rospy.sleep(0.1)

  """
    captures pose of the current wrist_link position and finds the transformation_w->t.
    param:
        tag_id  tag id of the marker
  """
  def capture_pose(self, tag_id):
    #pub = rospy.Publisher('move_base/goal', MoveBaseActionGoal, queue_size=10)

    # get the marker pose
    tag_id = int(tag_id)
    marker_pose_stamped = None   # this is a PostStamped
    for marker in self.reader.markers:
        if marker.id == tag_id:
            marker_pose_stamped = marker.pose
            break

    # current pose of the wrist_link
    wrist_pose = lookup_transform("base_link", "wrist_roll_link") # this is a Pose!
    wristPose = PoseStamped(pose=wrist_pose)  # this is also a PostStamped
    wristPose.header.frame_id = "/base_link"
    wristPose.header.stamp = rospy.Time.now()

    # get Transformation_b->m
    marker_pos = marker_pose_stamped.pose.position
    marker_ori = marker_pose_stamped.pose.orientation
    marker_in_base = np.dot(tft.translation_matrix([marker_pos.x, marker_pos.y, marker_pos.z]), tft.quaternion_matrix([marker_ori.x, marker_ori.y, marker_ori.z, marker_ori.w]))

    # get Transformation_b->w
    wrist_pos = wrist_pose.position
    wrist_ori = wrist_pose.orientation
    wrist_in_base = np.dot(tft.translation_matrix([wrist_pos.x, wrist_pos.y, wrist_pos.z]), tft.quaternion_matrix([wrist_ori.x, wrist_ori.y, wrist_ori.z, wrist_ori.w]))
    print(wrist_in_base)

    # get transformation_m->w (wrist in terms of marker)
    wrist_in_marker = np.dot(np.linalg.inv(marker_in_base), wrist_in_base)
    command = (tag_id, wrist_in_marker)
    print(command)
    self.list.append(command)

  def open_gripper(self):
    self.gripper.open()
    self.list.append("open")

  def close_gripper(self):
    self.gripper.close()
    self.list.append("close")

  def save_poses(self):
    myfile = open('pickle/' + self.name, 'w')  # just hard-coded for now
    pickle.dump(self.list, myfile)
    myfile.close()

  def relax(self):
    print("RELAXXXXINGGGG")
    self.arm.relax()

  def list_tags(self):
    for marker in self.reader.markers:
      print str(marker.id)
  
  class ArTagReader(object):
    def __init__(self):
        self.markers = []

    def callback(self, msg):
        self.markers = msg.markers
