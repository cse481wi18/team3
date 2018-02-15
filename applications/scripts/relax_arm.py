#!/usr/bin/env python
from fetch_api import Arm
import rospy

def wait_for_time():
    while rospy.Time().now().to_sec() == 0:
        pass
rospy.init_node("relaxxxxx")
wait_for_time()
a = Arm()
a.relax()
