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

def show_marker(marker_publisher, location):
    marker = Marker(
                type=Marker.LINE_STRIP,
                id=0,
                lifetime=rospy.Duration(0),
                pose=Pose(Point(0,0,0), Quaternion(0, 0, 0, 1)),
                scale=Vector3(0.1, 0.1, 0.1),
                header=Header(frame_id='odom'),
                color=ColorRGBA(0.0, 1.0, 0.0, 0.8),
                points=nav_path._path
                              )
    marker_publisher.publish(marker)


def show_text_in_rviz(marker_publisher, text):
    marker = Marker(
                type=Marker.TEXT_VIEW_FACING,
                id=0,
                lifetime=rospy.Duration(10),
                pose=Pose(Point(0.5, 0.5, 2), Quaternion(0, 1, 0, 1)),
                scale=Vector3(0.1, 0.1, 0.1),
                header=Header(frame_id='base_link'),
                color=ColorRGBA(0.0, 1.0, 0.0, 0.8),
                text="Hello, world!"
               )
    marker_publisher.publish(marker)

class NavPath(object):
    def __init__ (self):
        self._path=[]
    def callback(self,msg):
        rospy.loginfo(msg)
        pos = msg.pose.pose.position
        distance = math.sqrt(pos.x * pos.x + pos.y * pos.y + pos.z * pos.z)
        if distance>=0.01:
            self._path.append(pos)
            show_marker(marker_publisher, "Hello, world!")

def main():
    global nav_path
    global marker_publisher
    marker_publisher = rospy.Publisher('visualization_marker', Marker, queue_size=10) 
    rospy.sleep(0.5)                                                             
    nav_path = NavPath()
    rospy.Subscriber('odom', Odometry, nav_path.callback)
    rospy.init_node('my_node')
    show_marker(marker_publisher, nav_path._path)
    wait_for_time()
    rospy.spin()

if __name__ == '__main__':
    main()
