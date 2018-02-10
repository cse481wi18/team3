#!/usr/bin/env python
import action_saver_backend as backend

# Notes on backend methods:
#   open_gripper - just call the fetch api
#   close_gripper - just call the fetch api
#   capture_pose - the complicated one, should probably take a "reference point" argument but not sure how
#   save_poses(list) - pickle list to a file

def print_controls():
    print "Controls:"
    print "\tsave\t Capture current pose"
    print "\topen\t Open gripper"
    print "\tclose\t Close gripper"
    print "\tquit\t Finish and save"

def main():
    print "You're now recording an action"
    print "Please assign a name: "
    action_name = raw_input("> ")
    poses = []
    print "Recording action " + action_name
    print_controls()
    while True:
        user_input = raw_input("> ")
        user_input = user_input.lower().strip()
        if (user_input.startswith("s")):
            print "Capturing pose..."
            # figure out which marker to save it relative to
            # TODO: how do we specify markers
            pose = backend.capture_pose()
            poses.append(pose)
        elif user_input.startswith("o"):
            print "Opening gripper"
            backend.open_gripper()
        elif user_input.startswith("c"):
            print "Closing gripper"
            backend.close_gripper()
        elif user_input.startswith("q"):
            print "Saving action and exiting"
            backend.save_poses(poses)
            break
        else:
            print "Input not recognized."
            print_controls()
        
if __name__ == "__main__":
    main()
