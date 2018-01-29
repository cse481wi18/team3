import sys
import os
import pickle
from geometry_msgs.msg import PoseWithCovarianceStamped
import rospy
import geometry_msgs.msg
from nav_msgs.msg import Odometry
from pathlib import Path

class PoseCallback(object):

  def __init__(self):
    self.currentPos = None
    print 'in init'
    #self._pos_sub = rospy.Subscriber('amcl_pose', geometry_msgs.msg.PoseWithCovarianceStamped, self.callback)
    #TODO: fix this to use map instead of odom
    rospy.Subscriber('odom', Odometry, self.callback)

  def callback(self, msg):
    #print msg.amcl_pose.pose.pose.position
    #self.currentPos = msg.amcl_pose.pose.pose.position
    #print self.currentPos
    #self.currentPos = msg.pose.pose.position
    #print self.currentPos
    self.currentPos = msg

def printCommands():
  print 'Commands:'
  print 'list: List saved poses.'
  print 'save <name>: Save the robot\'s current pose as <name>. Overwrites if <name> already exists.'
  print 'delete <name>: Delete the pose given by <name>.'
  print 'goto <name>: Sends the robot to the pose given by <name>.'
  print 'help: Show this list of commands'


def main():
  print 'Welcome to the map annotator!'
  rospy.init_node('amcl', anonymous=True)
  printCommands();
  poseCallback = PoseCallback()
  while (True):
    sys.stdout.write('> ')
    line = sys.stdin.readline()
    lineComponents = line.split(' ')
    if lineComponents[0] == 'save':
      name = line[len(lineComponents[0]) + 1]
      if Path('pickle/' + name.strip()).is_file():
        print 'pose \'' + name + '\' already exists'
      else:
        print poseCallback.currentPos
        #print rospy.amcl.amcl_pose.pose.pose.position
        print name
        myfile = open('pickle/' + name.strip(), 'w')
        pickle.dump(poseCallback.currentPos, myfile)
    elif lineComponents[0] == 'delete':
      os.remove('pickle/' + lineComponents[1].strip()) 
    elif lineComponents[0].strip() == 'q':
      exit()
    elif lineComponents[0].strip() == 'list':
      files = os.listdir('pickle')
      for cfile in files:
        print cfile
    elif lineComponents[0].strip() == 'help':
      printCommands()
    elif lineComponents[0] == 'goto':
      pass      
    else:
      print 'unrecognized'


if __name__ == '__main__':
  main()
