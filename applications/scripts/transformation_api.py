#!/usr/bin/env python
import tf
import tf.transformations as tft
import math
import numpy as np
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Quaternion, Pose, PoseStamped

def lookup_transform(a, b):
    listener = tf.TransformListener()
    rospy.sleep(0.1)
    pose = None
    while pose is None:
        try:
            pose = listener.lookupTransform(a, b, rospy.Time())
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            # rospy.sleep(0.01)
            pass
    return Pose(Point(*pose[0]), Quaternion(*pose[1]))

def dot_poses(pose1, pose2):
    pos1 = pose1.position
    ori1 = pose1.orientation
    pos2 = pose2.position
    ori2 = pose2.orientation
    obj1_mtx = np.dot(tft.translation_matrix([pos1.x, pos1.y, pos1.z]), tft.quaternion_matrix([ori1.x, ori1.y, ori1.z, ori1.w]))
    obj2_mtx = np.dot(tft.translation_matrix([pos2.x, pos2.y, pos2.z]), tft.quaternion_matrix([ori2.x, ori2.y, ori2.z, ori2.w]))
    dot_mtx = np.dot(obj1_mtx, obj2_mtx)
    pose = Pose(Point(*tft.translation_from_matrix(dot_mtx)), Quaternion(*tft.quaternion_from_matrix(dot_mtx)))
    pose_stamped = PoseStamped()
    pose_stamped.header.frame_id = "/base_link"
    pose_stamped.header.stamp = rospy.Time.now()
    pose_stamped.pose = pose
    return pose_stamped
