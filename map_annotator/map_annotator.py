import sys
import os
import pickle
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import rospy
import geometry_msgs.msg
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseGoal, MoveBaseActionGoal

class PoseCallback(object):

  def __init__(self):
    self.currentPos = None
    #self._pos_sub = rospy.Subscriber('amcl_pose', geometry_msgs.msg.PoseWithCovarianceStamped, self.callback)
    #TODO: fix this to use map instead of odom
    rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.callback)

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

def printCommands():
  print 'Commands:'
  print 'list: List saved poses.'
  print 'save <name>: Save the robot\'s current pose as <name>. Overwrites if <name> already exists.'
  print 'delete <name>: Delete the pose given by <name>.'
  print 'goto <name>: Sends the robot to the pose given by <name>.'
  print 'help: Show this list of commands'


def main():
  print 'Welcome to the map annotator!'
  pub = rospy.Publisher('move_base/goal', MoveBaseActionGoal, queue_size=10)
  rospy.init_node('amcl', anonymous=True)
  printCommands();
  poseCallback = PoseCallback()
  while (True):
    sys.stdout.write('> ')
    line = sys.stdin.readline()
    lineComponents = line.split(' ')
    if lineComponents[0] == 'save':
      name = line[(len(lineComponents[0]) + 1):]
      #print poseCallback.currentPos
      #print rospy.amcl.amcl_pose.pose.pose.position
      #print name
      myfile = open('pickle/' + name.strip(), 'w')
      pickle.dump(poseCallback.currentPos, myfile)
      myfile.close()
    elif lineComponents[0] == 'delete':
      name = line[(len(lineComponents[0]) + 1):].strip()
      if not os.path.exists('pickle/' + name):
        print 'No such pose \'' + name + '\''
      else:
        os.remove('pickle/' + name)
    elif lineComponents[0].strip() == 'q':
      exit()
    elif lineComponents[0].strip() == 'list':
      files = os.listdir('pickle')
      if len(files) == 0:
        print 'No poses'
      else:
        print 'Poses:'
        for cfile in files:
          print '  ' + cfile
    elif lineComponents[0].strip() == 'help':
      printCommands()
    elif lineComponents[0] == 'goto':
      name = line[(len(lineComponents[0]) + 1):].strip()
      if not os.path.exists('pickle/' + name):
        print 'No such pose \'' + name + '\''
      else:
        loadfile = open('pickle/' + name, 'rb')
        stampedCoPose = pickle.load(loadfile)
        loadfile.close()
        mbagoal = MoveBaseActionGoal()
        mbgoal = MoveBaseGoal()
        mbgoal.target_pose = stampedCoPose #potential issue here
        mbagoal.goal = mbgoal
        #mbagoal.header =
        #mbagoal.goal_id =
        pub.publish(mbagoal)
    else:
      print 'Unrecognized command'


if __name__ == '__main__':
  main()
