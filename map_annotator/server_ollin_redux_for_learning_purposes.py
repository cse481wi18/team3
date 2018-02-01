import rospy



def print_intro():
    rospy.loginfo("Welcome to the map annotator!")
    rospy.loginfo("Commands:")
    rospy.loginfo("  list: List saved poses.")
    rospy.loginfo("  list: List saved poses.")
    rospy.loginfo("  save <name>: Save the robot's current pose as <name>. Overwrites if <name> already exists.")
    rospy.loginfo("  delete <name>: Delete the pose given by <name>.")
    rospy.loginfo("  goto <name>: Sends the robot to the pose given by <name>.")
    rospy.loginfo("  help: Show this list of commands")

def main():
    print_intro():
    while true:
        command, arg = raw_input("> ").split(' ', 1)
        if command.startswith("help"):
            print_intro()
        elif command.startswith("list"):
            list_poses()
        elif command.startswith("save"):
            save_pose(arg)

            

if __name__ == "__main__":
    main()
