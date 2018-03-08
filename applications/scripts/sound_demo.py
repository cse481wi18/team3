#! /usr/bin/env python

import fetch_api
import rospy
from geometry_msgs.msg import Twist

class PlanCallback(object):
    def __init__(self):
        self.playing = False
        rospy.Subscriber('cmd_vel', Twist, self.callback)

    def callback(self, msg):
        global sound
        if msg.angular.z > 0.7:
            if not self.playing:
                self.playing = True
                sound.say('sharp left')
        elif msg.angular.z < -0.7:
            if not self.playing:
                self.playing = True
                sound.say('sharp right')
        else:
            self.playing = False
        print 'callback:', msg

rospy.init_node('speak')
while rospy.Time().now().to_sec() == 0:
    pass

sound = fetch_api.RobotSound()
cbk = PlanCallback()
rospy.spin()
