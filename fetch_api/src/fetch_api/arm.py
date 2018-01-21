import actionlib
from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from trajectory_msgs.msg import JointTrajectory
import rospy

from .arm_joints import ArmJoints

ACTION_NAME = "arm_controller/follow_joint_trajectory"
TIME_FROM_START = 5

class Arm(object):
    """Arm controls the robot's arm.

    Joint space control:
        joints = ArmJoints()
        # Fill out joint states
        arm = fetch_api.Arm()
        arm.move_to_joints(joints)
    """

    def __init__(self):
	self.client = actionlib.SimpleActionClient(ACTION_NAME, FollowJointTrajectoryAction)
	rospy.logerr("initialized client")
	self.client.wait_for_server()
	rospy.logerr("got server response")

    def move_to_joints(self, arm_joints):
        """Moves the robot's arm to the given joints.

        Args:
            arm_joints: An ArmJoints object that specifies the joint values for
                the arm.
        """
	rospy.logerr("setting arm joints")
        # Create a trajectory point
	point = JointTrajectoryPoint()
        # Set position of trajectory point
	point.positions.extend(arm_joints.values())
        # Set time of trajectory point
	point.time_from_start = rospy.Duration(TIME_FROM_START)

        # Create goal
	goal = FollowJointTrajectoryGoal()
        # Add joint name to list
	trajectory = JointTrajectory()
	trajectory.joint_names.extend(ArmJoints.names())
        # Add the trajectory point created above to trajectory
	trajectory.points.append(point)

        # Send goal
	goal.trajectory = trajectory
	self.client.send_goal(goal)
	rospy.logerr("sent move to joint")
        # Wait for result
	self.client.wait_for_result()
        rospy.logerr('joint position set')

