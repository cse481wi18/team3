#! /usr/bin/env python

import fetch_api
import rospy
from geometry_msgs.msg import Twist

class PlanCallback(object):
    def __init__(self):
        self.playing = False
        self.prev = 0.0
        rospy.Subscriber('cmd_vel', Twist, self.callback)

    def callback(self, msg):
        cap = 0.7
        global sound
        if msg.angular.z > cap:
            if self.prev > cap and not self.playing:
                self.playing = True
                sound.say('left turn')
        elif msg.angular.z < -cap:
            if self.prev < -cap and not self.playing:
                self.playing = True
                sound.say('right turn')
        else:
            self.playing = False
        self.prev = msg.angular.z
        print 'callback:', msg

rospy.init_node('speak')
while rospy.Time().now().to_sec() == 0:
    pass

sound = fetch_api.RobotSound()
cbk = PlanCallback()
rospy.spin()
