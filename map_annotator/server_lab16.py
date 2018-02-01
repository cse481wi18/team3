#!/usr/bin/env python
import sys
import os
import pickle
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import rospy
import geometry_msgs.msg
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseGoal, MoveBaseActionGoal
from map_annotator.msg import UserAction, PoseNames

# subscribes to user's command and executes them
class UserActionSubscriber(object):
  def __init__(self, mbg_pub, pose_name_pub, pose):
    rospy.logerr("SUBSCRIBER CREATED")
    rospy.Subscriber('map_annotator/user_actions', UserAction, self.callback)
    # this might not work
    self.pub = mbg_pub
    self.pose_pub = pose_name_pub
    self.poseCallback = pose
    rospy.logerr(os.getcwd())
    if not os.path.exists("pickle"):
      os.makedirs("pickle")

  def publish_poses(self):
     self.pose_pub.publish(os.listdir('pickle'))

  def callback(self, msg):
    rospy.logerr(msg)
    print("WE GOT A MESSAGE!")
    cmd = msg.command
    name = msg.name
    rospy.logerr("Comparing " + str(cmd) + " to " + str(msg.CREATE))
    if cmd == msg.CREATE:
      myfile = open("pickle/" + name, 'w+')
      pickle.dump(self.poseCallback.currentPos, myfile)
      myfile.close()
      self.publish_poses()
    elif cmd == msg.DELETE:
      if not os.path.exists("pickle/" + name):
        print 'No such pose \'' + name + '\''
      else:
        os.remove("pickle/" + name)
      self.publish_poses()
    elif cmd == msg.GOTO:
      name = line[(len(lineComponents[0]) + 1):].strip()
      if not os.path.exists("pickle/" + name):
        print 'No such pose \'' + name + '\''
      else:
        loadfile = open("pickle/" + name, 'rb')
        stampedCoPose = pickle.load(loadfile)
        loadfile.close()
        mbagoal = MoveBaseActionGoal()
        mbgoal = MoveBaseGoal()
        mbgoal.target_pose = stampedCoPose #potential issue here
        mbagoal.goal = mbgoal
        self.pub.publish(mbagoal)

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
