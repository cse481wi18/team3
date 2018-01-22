#!/usr/bin/env python

import fetch_api
import math
import rospy
from web_teleop.srv import SetTorso, SetArm, SetHead, SetGripper, SetTorsoResponse, SetArmResponse, SetHeadResponse, SetGripperResponse


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass

def toRadians(degrees):
    return degrees * math.pi / 180

class ActuatorServer(object):
    def __init__(self):
        self._torso = fetch_api.Torso()
        self._arm = fetch_api.Arm()
        self._gripper = fetch_api.Gripper()
        self._head = fetch_api.Head()

    def handle_set_torso(self, request):
        # move the torso to the requested height
    	self._torso.set_height(request.height)
        return SetTorsoResponse()

    def handle_set_arm(self, request):
        # move the arm to the requested joints 
        jointsRadians = map(toRadians, request.joints)
        print("Received joint angles: " + str(jointsRadians))
    	self._arm.move_to_joints(fetch_api.ArmJoints.from_list(jointsRadians))
        return SetArmResponse()

    def handle_set_head(self, request):
        self._head.pan_tilt(toRadians(request.pan), toRadians(request.tilt))
        return SetHeadResponse()

    def handle_set_gripper(self, request):
        if request.state == "open":
            self._gripper.open()
        else:
            self._gripper.close()
        return SetGripperResponse()

def main():
    rospy.init_node('web_teleop_actuators')
    wait_for_time()
    server = ActuatorServer()

    torso_service = rospy.Service('web_teleop/set_torso', SetTorso, server.handle_set_torso)

    arm_service = rospy.Service('web_teleop/set_arm', SetArm, server.handle_set_arm)
    gripper_service = rospy.Service('web_teleop/set_gripper', SetGripper, server.handle_set_gripper)

    head_service = rospy.Service('web_teleop/set_head', SetHead, server.handle_set_head)

    rospy.spin()


if __name__ == '__main__':
    main()
