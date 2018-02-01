#!/usr/bin/env python
import sys
import os
import pickle
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Pose, Point, Quaternion
import rospy
import geometry_msgs.msg
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseGoal, MoveBaseActionGoal
from map_annotator.msg import UserAction, PoseNames
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback, Marker

# subscribes to user's command and executes them
class UserActionSubscriber(object):
  def __init__(self, mbg_pub, pose_name_pub, pose):
    rospy.Subscriber('map_annotator/user_actions', UserAction, self.callback)
    self.pub = mbg_pub
    self.pose_name_pub = pose_name_pub
    self.poseCallback = pose
    self.poses = {}
    # TODO: use the server...
    self.marker_server = InteractiveMarkerServer("map_annotator/map_poses")
    if os.path.exists("poses.pkl"):
      pickle_file = open("poses.pkl", "r")
      self.poses = pickle.load(pickle_file)
      pickle_file.close() 
    self.publish_poses()

  def handle_marker_event(self, event):
    #rospy.logerr("Received marker event" + str(event))
    #rospy.logerr(dir(event))
    #rospy.logerr(help(event))
    self.poses[event.marker_name] = event.pose
    if event.event_type == 5:
      self._save_poses()
      self.publish_poses()

  def _create_marker(self, pose, pose_name):
    # TODO: make a marker
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "map"
    int_marker.name = pose_name
    int_marker.description = pose_name
    int_marker.pose.position = pose.position
    int_marker.pose.position.z += 0.1
    int_marker.pose.orientation = pose.orientation

    box_marker = Marker()
    box_marker.type = Marker.ARROW
    box_marker.pose.orientation.w = 1
    box_marker.scale.x = 0.45
    box_marker.scale.y = 0.05
    box_marker.scale.z = 0.05
    box_marker.color.r = 0.0
    box_marker.color.g = 0.5
    box_marker.color.b = 0.5
    box_marker.color.a = 1.0

    #click_control = InteractiveMarkerControl()
    #click_control.interaction_mode = InteractiveMarkerControl.BUTTON
    #click_control.orientation.w = 1
    #click_control.orientation.x = 0
    #click_control.orientation.y = 1
    #click_control.orientation.z = 0
    #click_control.always_visible = True
    #click_control.markers.append(box_marker)
    #int_marker.controls.append(click_control)

    button_control = InteractiveMarkerControl()
    button_control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
    button_control.orientation.w = 1
    button_control.orientation.x = 0
    button_control.orientation.y = 1
    button_control.orientation.z = 0
    button_control.always_visible = True
    button_control.markers.append(box_marker)
    int_marker.controls.append(button_control)

    button_control2 = InteractiveMarkerControl()
    button_control2.orientation.w = 1
    button_control2.orientation.x = 0
    button_control2.orientation.y = 1
    button_control2.orientation.z = 0
    button_control2.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    button_control2.always_visible = True
    int_marker.controls.append(button_control2)

    self.marker_server.insert(int_marker, self.handle_marker_event)
    self.marker_server.applyChanges()

  def publish_poses(self):
    self.pose_name_pub.publish(list(self.poses.keys()))
    for pose_name in self.poses:
      pose = self.poses[pose_name]
      self._create_marker(pose, pose_name)
    self.marker_server.applyChanges()
  def _save_poses(self):
    pickle_file = open("poses.pkl", 'w+')
    pickle.dump(self.poses, pickle_file)
    pickle_file.close()

  def goto(self, name):
    # TODO: Make this use a simpleActionClient
    mbagoal = MoveBaseActionGoal()
    mbgoal = MoveBaseGoal()
    mbgoal.target_pose = PoseStamped(pose=self.poses[name])
    mbgoal.target_pose.header.frame_id = "/map"
    mbgoal.target_pose.header.stamp = rospy.Time.now()
    mbagoal.goal = mbgoal
    self.pub.publish(mbagoal)

  def callback(self, msg):
    rospy.logerr(msg)
    cmd = msg.command
    name = msg.name
    if cmd == msg.CREATE:   
      self.poses[name] = Pose()
      self.publish_poses()
      self._save_poses()
    elif cmd == msg.DELETE:
      del self.poses[name]
      self.marker_server.erase(name)
      self._save_poses()
      self.publish_poses()
    elif cmd == msg.GOTO:
      rospy.logerr("Received call to goto with name " + name)
      self.goto(name)
# reports current position of the robot
class PoseCallback(object):
  def __init__(self):
    self.currentPos = None
    #self._pos_sub = rospy.Subscriber('amcl_pose', geometry_msgs.msg.PoseWithCovarianceStamped, self.callback)
    #TODO: fix this to use map instead of odom
    rospy.Subscriber('odom', Odometry, self.callback)

  def callback(self, msg):
    #print msg.amcl_pose.pose.pose.position
    #self.currentPos = msg.amcl_pose.pose.pose.position
    #print self.currentPos
    #self.currentPos = msg.pose.pose.position
    #print self.currentPos
    pose = PoseStamped()
    pose.pose = msg.pose.pose
    pose.header = msg.header
    self.currentPos = pose

def main():
  rospy.logerr('Initializing pose name & move base goal publisher ...')
  pose_name_pub = rospy.Publisher('map_annotator/pose_names', PoseNames, queue_size=10, latch=True)
  mbg_pub = rospy.Publisher('move_base/goal', MoveBaseActionGoal, queue_size=10)

  rospy.logerr('Initializing node...')
  rospy.init_node('amcl', anonymous=True)
  rospy.logerr('Initializing user action subscriber...')
  poseCallback = UserActionSubscriber(mbg_pub, pose_name_pub, PoseCallback())
  rospy.sleep(0.5)
  rate = rospy.Rate(10)
  rospy.logerr('Finished Setup')
  rospy.spin()


if __name__ == '__main__':
  main()
