#! /usr/bin/env python

from threading import Thread
from fetch_api import Arm, Torso, Base
import rospy
import math
from geometry_msgs.msg import Pose, PoseStamped

def wait_for_time():
    while rospy.Time().now().to_sec() == 0:
        pass

def get_pose(position, orientation):
    pose = Pose()
    pose.position.x = position[0]
    pose.position.y = position[1]
    pose.position.z = position[2]
    pose.orientation.x = orientation[0]
    pose.orientation.y = orientation[1]
    pose.orientation.z = orientation[2]
    pose.orientation.w = orientation[3]
    return pose

def get_pose_stamped(pose, frame_id='base_link'):
    ps = PoseStamped(pose=pose)
    ps.header.frame_id = frame_id
    ps.header.stamp = rospy.Time.now()
    return ps

GRIPPER_INIT_POSE = ([0.05629574200314329, -0.1758155809789218, 1.0147567021849901], [-0.7100140419364234, 0.041508636572643066, -0.7025522183479911, -0.02403068532729804])

GRIPPER_START_NAV_POSE = ([-0.28090997697643605, 0.2462498538252318, 1.1029694087072481], [-0.7065367445526678, 0.04637212466308408, -0.7057492305604214, -0.023947405173068113])

def get_init_gripper_pose():
    return get_pose(*GRIPPER_INIT_POSE)

def get_start_nav_gripper_pose():
    return get_pose(*GRIPPER_START_NAV_POSE)

def move_to_init_pose():
    rospy.sleep(0.5)
    arm = Arm()
    pose = get_init_gripper_pose()
    b = Base()
    do_in_parallel(lambda: b.turn(-math.pi), lambda: arm.move_to_pose(get_pose_stamped(pose)))

def move_to_start_nav_pose():
    rospy.sleep(0.5)
    arm = Arm()
    pose = get_start_nav_gripper_pose()
    b = Base()
    do_in_parallel(lambda: b.turn(math.pi), lambda: arm.move_to_pose(get_pose_stamped(pose)))

def do_in_parallel(*args):
    t_list = list()
    for arg in args:
        t = Thread(target=arg)
        t_list.append(t)
        t.start()
    for t in t_list:
        t.join()


if __name__ == '__main__':
    rospy.init_node("common_pose")
    wait_for_time()
    move_to_start_nav_pose()
    move_to_init_pose()
    rospy.sleep(5)
