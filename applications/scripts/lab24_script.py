#!/usr/bin/env python
import tf
import rospy
import tf.transformations as tft
import numpy as np

rospy.init_node('lab24_script')
listener = tf.TransformListener()
rospy.sleep(0.1)

def matrix_from_position_and_orientation(matrix):
  return (tft.translation_from_matrix(matrix), tft.quaternion_from_matrix(matrix))

def position_and_orientation_to_matrix(position, orientation):
  return np.dot(tft.translation_matrix(position), tft.quaternion_matrix(orientation))

base_link_to_object = ((0.6, -0.1, 0.7), (0,0,0.38268343,0.92387953))

print "base link to object transform is " + str(base_link_to_object)
# (it's a position and then an orientation tuple)

object_to_pregrasp = ((-0.1, 0, 0), (0, 0, 0, 1))

base_link_to_object_matrix = position_and_orientation_to_matrix(*base_link_to_object)
print "base link to object matrix is\n" + str(base_link_to_object_matrix)
object_to_pregrasp_matrix = position_and_orientation_to_matrix(*object_to_pregrasp)
print "pregrasp to object matrix is\n" + str(object_to_pregrasp_matrix)

base_link_to_pregrasp_matrix = np.dot(base_link_to_object_matrix, object_to_pregrasp_matrix)
print "dot product of those two matrices is\n" + str(base_link_to_pregrasp_matrix)

base_link_to_pregrasp = matrix_from_position_and_orientation(base_link_to_pregrasp_matrix)

print "base link to pregrasp is\n" + str(base_link_to_pregrasp)
