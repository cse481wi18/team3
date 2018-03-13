import rospy
import fetch_api

rospy.init_node("config")
while rospy.Time().now().to_sec() == 0:
    pass

torso = fetch_api.Torso()
torso.set_height(0.4)

gripper = fetch_api.Gripper()
gripper.close()

head = fetch_api.Head()
head.pan_tilt(0, 0)
