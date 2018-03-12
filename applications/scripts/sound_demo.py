#! /usr/bin/env python

import fetch_api
import rospy
from geometry_msgs.msg import Twist

alpha = 0.5

class PlanCallback(object):

    def __init__(self):
        self.playing = False
        self.prev = 0.0
        rospy.Subscriber('cmd_vel', Twist, self.callback)
        self.angular_z = 0.0

    def callback(self, msg):
        global alpha
        self.angular_z = alpha * self.angular_z + (1 - alpha) * msg.angular.z
        cap = 0.48
        global sound
        if self.angular_z > cap:
            if self.prev > cap and not self.playing:
                self.playing = True
                sound.say('left turn')
        elif self.angular_z < -cap:
            if self.prev < -cap and not self.playing:
                self.playing = True
                sound.say('right turn')
        else:
            self.playing = False
        self.prev = self.angular_z
        print 'callback:', msg

rospy.init_node('speak')
while rospy.Time().now().to_sec() == 0:
    pass

sound = fetch_api.RobotSound()
cbk = PlanCallback()
rospy.spin()
