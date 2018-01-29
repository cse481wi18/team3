#import ????????_msgs.msg
import math
import copy
import tf.transformations as tft
import numpy as np
import rospy
from geometry_msgs.msg import Twist, Vector3, Quaternion
from nav_msgs.msg import Odometry

def quaternion_to_yaw(q):
    m = tft.quaternion_matrix([q.x, q.y, q.z, q.w])
    x = m[0,0]
    y = m[1,0]
    theta_rad = math.atan2(y,x)
    return theta_rad

def compute_desired_yaw(curr_loc, desired_loc):
    dx = desired_loc.x - curr_loc.x
    dy = desired_loc.y - curr_loc.y
    theta_rad = math.atan2(dy, dx)
    curr_rad = quaternion_to_yaw(curr_loc)
    return theta_rad - curr_rad

class Base(object):
    """Base controls the mobile base portion of the Fetch robot.

            Sample usage:
            base = fetch_api.Base()
            while CONDITION:
                base.move(0.2, 0)
                base.stop()
    """

    def __init__(self):
        self.pub = rospy.Publisher('cmd_vel', Twist)
        self._odom_sub = rospy.Subscriber('odom', Odometry, callback=self._odom_callback)
        self.message = None

    def _odom_callback(self, msg):
        rospy.loginfo(msg.header)
        self.message = msg

    def move(self, linear_speed, angular_speed):
        """Moves the base instantaneously at given linear and angular speeds.

        "Instantaneously" means that this method must be called continuously in
            a loop for the robot to move.

        Args:
            linear_speed: The forward/backward speed, in meters/second. A
            positive value means the robot should move forward.
            angular_speed: The rotation speed, in radians/second. A positive
            value means the robot should rotate clockwise.
        """
        self.pub.publish(Twist(Vector3(linear_speed, 0, 0), Vector3(0, 0, angular_speed)))
        rospy.logerr('Set velocity to x: ' + str(linear_speed) + " angle z: " + str(angular_speed));

    def stop(self):
        """Stops the mobile base from moving.
        """
        self.pub.publish(Twist(Vector3(0, 0, 0), Vector3(0, 0, 0)))
        rospy.logerr('Set velocity to 0.');

    def go_forward(self, distance, speed=0.1):
        """Moves the robot a certain distance.

        It's recommended that the robot move slowly. If the robot moves too
        quickly, it may overshoot the target. Note also that this method does
        not know if the robot's path is perturbed (e.g., by teleop). It stops
        once the distance traveled is equal to the given distance or more.

        Args:
        distance: The distance, in meters, to move. A positive value
        means forward, negative means backward.
        speed: The speed to travel, in meters/second.
        """
        # TODO: rospy.sleep until the base has received at least one message on /odom
        # TODO: record start position, use Python's copy.deepcopy
        while self.message is None:
            rospy.sleep(1.)
        
        start = copy.deepcopy(self.message.pose.pose)
        cur_pos = start.position.x
        rate = rospy.Rate(10)

        # TODO: CONDITION should check if the robot has traveled the desired distance
        # TODO: Be sure to handle the case where the distance is negative!
        print 'BEFORE LOOP:'
        print cur_pos
        print self.message.pose.pose.position.x
        while abs((cur_pos - self.message.pose.pose.position.x)) < abs(distance):
            # TODO: you will probably need to do some math in this loop to check the CONDITION
            print "cur:" + str(cur_pos) + " goes:" + str(cur_pos - self.message.pose.pose.position.x) + " distance:" + str(distance)
            direction = -1 if distance < 0 else 1
            self.move(direction * speed, 0)
            cur_pos += direction * speed
            rate.sleep()
    
    def turn(self, angular_distance, speed=0.5):
        """Rotates the robot a certain angle.
        Args:
        angular_distance: The angle, in radians, to rotate. A positive
                                    value rotates counter-clockwise.
        speed: The angular speed to rotate, in radians/second.
        """
        # rospy.sleep until the base has received at least one message on /odom
        print 'IN TURN!!!'
        while self.message is None:
            rospy.sleep(1.)
        # record start position, use Python's copy.deepcopy
        start = copy.deepcopy(self.message.pose.pose)
        cur_rad = quaternion_to_yaw(start.orientation)
        #desired_rad = (cur_rad + angular_distance) % (2 * math.pi)
        # TODO: What will you do if angular_distance is greater than 2*pi or less than -2*pi?
        #angular_distance = angular_distance % (2 * math.pi)
        rate = rospy.Rate(10)
        print 'BEFORE WHILE!!'
        # TODO: CONDITION should check if the robot has rotated the desired amount
        # TODO: Be sure to handle the case where the desired amount is negative!
        while abs((cur_rad - (quaternion_to_yaw(self.message.pose.pose.orientation)))) < abs(angular_distance):
            # TODO: you will probably need to do some math in this loop to check the CONDITION
            direction = -1 if angular_distance < 0 else 1
            cur_rad += (direction * speed)
            rospy.logerr("cur:" + str(cur_rad) + " goes:" + str(cur_rad - (quaternion_to_yaw(self.message.pose.pose.orientation))) + " distance:" + str(angular_distance))
            self.move(0, direction * speed)
            rate.sleep()
