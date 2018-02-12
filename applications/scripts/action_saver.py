#!/usr/bin/env python
import actionlib
import action_saver_backend as asb
import rospy
import robot_controllers_msgs.msg
from robot_controllers_msgs.msg import QueryControllerStatesAction, QueryControllerStatesGoal, ControllerState

"""
Saves the pose when the person moves the arm.
Goal:
    get the transformation from base_link -> tag
    get the transformation from base_link -> wrist_link
    get the transformation from wrist_link -> tag = T_b->t * T_b->w
"""


# Notes on backend methods:
#   open_gripper - just call the fetch api
#   close_gripper - just call the fetch api
#   capture_pose - the complicated one, should probably take a "reference point" argument but not sure how
#   save_poses(list) - pickle list to a file

def wait_for_time():
    while rospy.Time().now().to_sec() == 0:
        pass


"""
    prints controls for user
"""
def print_controls():
    print "Controls:"
    print "\tsave\t Capture current pose"
    print "\topen\t Open gripper"
    print "\tclose\t Close gripper"
    print "\tquit\t Finish and save"


def main():
    print "You're now recording an action"
    # retrieves name of the pose
    print "Please assign a name: "
    action_name = raw_input("> ")
    print "Recording action " + action_name
    # get action saver from action_saver_backend.py
    backend = asb.actionSaver(action_name)
    print_controls()
    backend.relax()
    while True:
        # get user command
        user_input = raw_input("> ")
        user_input = user_input.lower().strip()
        # if the input is a save
        if (user_input.startswith("s")):
            print "Capturing pose..."
            # figure out which marker to save it relative to
            backend.list_tags() # prints each marker visible
            print "Choose one tag to save the position relative to"
            # get tag id of marker
            tag_id = raw_input("> ")
            tag_id = tag_id.strip()
            # saves transformation from wrist relative to the marker in backend cod3e
            backend.capture_pose(tag_id)
        # if the user wants to open gripper
        elif user_input.startswith("o"):
            print "Opening gripper"
            # opens gripper
            backend.open_gripper()
        # if the user wants to close gripper
        elif user_input.startswith("c"):
            print "Closing gripper"
            # closes gripper
            backend.close_gripper()
        elif user_input.startswith("q"):
            print "Saving action and exiting"
            backend.save_poses()
            break
        elif user_input.startswith("e"):
            break
        else:
            print "Input not recognized."
            print_controls()
        
if __name__ == "__main__":
    main()
