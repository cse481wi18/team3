#! /usr/bin/env python

import math
import copy
import rospy
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry

import tf.transformations as tft

class Base(object):
    """Base controls the mobile base portion of the Fetch robot.

    Sample usage:
        base = fetch_api.Base()
        while CONDITION:
            base.move(0.2, 0)
        base.stop()
    """

    def __init__(self):
        # Publish Twist (speed + angular speed)
        self.pub = rospy.Publisher('cmd_vel', Twist)
        # Subscribe to odometry
        self._odom_sub = rospy.Subscriber('odom', Odometry, callback=self._odom_callback)
        # Copy of the most recent odometry message
        self._latest_odom = None

    def _odom_callback(self, msg):
        self._latest_odom = msg

# Lab 2-3

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
        # rospy.logerr('Set velocity to x: ' + str(linear_speed) + " angle z: " + str(angular_speed));

    def stop(self):
        """Stops the mobile base from moving.
        """
        self.pub.publish(Twist(Vector3(0, 0, 0), Vector3(0, 0, 0)))
        rospy.logerr('Set velocity to 0.');

# Lab 4

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

        # Sleep until we've received at least one message from /odom
        while (self._latest_odom is None):
            rospy.sleep(0.1)

        # Deep copy the start position so that we have a fixed reference point
        start_pose = copy.deepcopy(self._latest_odom.pose.pose)
        start_x = start_pose.position.x

        # Local function to check if we're "there yet"
        def robot_has_gone_forward_full_distance():
            current_x = self._latest_odom.pose.pose.position.x
            current_x_displacement = current_x - start_x
            # Check that we've displaced by at least the distance requested
            return abs(current_x_displacement) >= abs(distance)

        # Sampling rate for position updates
        rate = rospy.Rate(10)

        # Keep moving by <speed> until we've traveled the full distance
        # Sampling every <rate>
        while (not robot_has_gone_forward_full_distance()):
            direction = -1 if distance < 0 else 1
            self.move(direction * speed, 0)
            rate.sleep()

    def turn(self, angular_distance, speed=0.5):
        """Rotates the robot a certain angle.

        Args:
            angular_distance: The angle, in radians, to rotate. A positive
                value rotates counter-clockwise.
            speed: The angular speed to rotate, in radians/second.
        """

        # Sleep until we've received at least one message from /odom
        while (self._latest_odom is None):
            rospy.sleep(0.1)

        # Deep copy the start position so that we have a fixed reference point
        start_pose = copy.deepcopy(self._latest_odom.pose.pose)

        # Orientation is a quaternion; we need to convert it to get yaw
        def quaternion_to_yaw(q):
            rotation_matrix = tft.quaternion_matrix([q.x, q.y, q.z, q.w])
            x = rotation_matrix[0, 0] # The x value of the x-axis (first column)
            y = rotation_matrix[1, 0] # The y value of the x-axis
            theta_rads = math.atan2(y, x) % (2 * math.pi) # Radians in [0, 2pi]
            return theta_rads

        start_yaw = quaternion_to_yaw(start_pose.orientation)

        def remaining_angle():
            # (see canvas for explanation)
            current_yaw = quaternion_to_yaw(self._latest_odom.pose.pose.orientation)
            goal_yaw = start_yaw + angular_distance
            if angular_distance > 0:
                distance_left = (goal_yaw - current_yaw) % (2 * math.pi)
            else:
                distance_left = (current_yaw - goal_yaw) % (2 * math.pi)
            return distance_left

        # Local function to check if we're "there yet"
        def robot_has_turned_full_distance():
            epsilon = 0.05
            return remaining_angle() < epsilon

        # Sampling rate for position updates
        rate = rospy.Rate(10)

        counter = 0

        # Keep moving by <speed> until we've traveled the full distance
        # Sampling every <rate>
        while (not robot_has_turned_full_distance() and counter <= 20):
            counter += 1
            print(remaining_angle())
            direction = -1 if angular_distance < 0 else 1
            speed = max(0.25, min(speed, remaining_angle()))
            self.move(0, direction * speed)
            rate.sleep()
