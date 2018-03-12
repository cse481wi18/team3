#! /usr/bin/env python

import fetch_api
import rospy
import time
from geometry_msgs.msg import Twist

alpha = 0.8
min_message_time_ms = 500

def millis():
    return int(round(time.time() * 1000))

class PlanCallback(object):

    def __init__(self):
        self.playing = False
        self.prev = 0.0
        rospy.Subscriber('cmd_vel', Twist, self.callback)
        self.angular_z = 0.0
        self.prev_message_time = 0

    def callback(self, msg):
        global alpha
        self.angular_z = alpha * self.angular_z + (1 - alpha) * msg.angular.z
        cap = 0.48
        global sound
        if self.angular_z > cap:
            #print "angular z: " + str(self.angular_z) + " > " + str(cap)
            if self.prev > cap and not self.playing:
                if (millis() - self.prev_message_time > min_message_time_ms):
                    self.playing = True
                    sound.say('left turn')
                    print "angular z: " + str(self.angular_z) + " > " + str(cap) + " -> left turn"
                    self.prev_message_time = millis()
        elif self.angular_z < -cap:
            #print "angular z: " + str(self.angular_z) + " < " + str(-cap)
            if self.prev < -cap and not self.playing:
                if (millis() - self.prev_message_time > min_message_time_ms):
                    self.playing = True
                    sound.say('right turn')
                    print "angular z: " + str(self.angular_z) + " < " + str(-cap) + " -> right turn"
                    self.prev_message_time = millis()
        else:
            #print "angular z: " + str(self.angular_z) + " -> stationary"
            self.playing = False
        self.prev = self.angular_z
        #print 'callback:', msg

rospy.init_node('speak')
while rospy.Time().now().to_sec() == 0:
    pass

sound = fetch_api.RobotSound()
cbk = PlanCallback()
rospy.spin()
