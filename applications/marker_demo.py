#!/usr/bin/env python
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA
from nav_msgs.msg import Odometry
import rospy
import math

def wait_for_time():                                              
    """Wait for simulated time to begin.                          
    """                                                           
    while rospy.Time().now().to_sec() == 0:                       
        pass

class NavPath(object):
    # Store a path (visualization state) and a reference to the publisher object
    def __init__ (self, publisher):
        self._path = []
        self._publisher = publisher
    # On odometry updates:
    # * Update the state
    # * Publish the new state
    def callback(self, msg):
        pos = msg.pose.pose.position
        distance = math.sqrt(pos.x * pos.x + pos.y * pos.y + pos.z * pos.z)
        # Don't publish points if the robot isn't moving
        if distance >= 0.01:
            self._path.append(pos)
            self.drawPath()
    def drawPath(self):
        # Create LINE_STRIP marker with 0 translation/rotation offset
        # in the "odometry" reference coordinate frame
        marker = Marker(
                    type = Marker.LINE_STRIP,
                    id = 0,
                    lifetime = rospy.Duration(0),
                    pose = Pose(Point(0,0,0), Quaternion(0, 0, 0, 1)),
                    scale = Vector3(0.1, 0.1, 0.1),
                    header = Header(frame_id='odom'),
                    color = ColorRGBA(1.0, 1.0, 1.0, 0.8),
                    points = self._path
                    )
        self._publisher.publish(marker)

def main():
    rospy.init_node('nav_path_display')
    wait_for_time()
    marker_publisher = rospy.Publisher('visualization_marker', Marker, queue_size=10) 
    # Wait for other nodes to *subscribe* before we start broadcasting!
    rospy.sleep(0.5)                                                             
    nav_path = NavPath(marker_publisher)
    rospy.Subscriber('odom', Odometry, nav_path.callback)
    rospy.spin()

if __name__ == '__main__':
    main()
