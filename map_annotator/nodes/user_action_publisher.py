#!/usr/bin/env python
import rospy
from map_annotator import UserAction

def wait_for_time():
    """Wait for simulated time to begin
    """
    while rospy.Time().now().to_sec() == 0:
        pass

def main():
    rospy.init_node('user_action_publisher')
    wait_for_time()
    #user_action_pub = rospy.Publisher('map_annotator/user_actions', 
