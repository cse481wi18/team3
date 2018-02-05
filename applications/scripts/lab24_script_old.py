#!/usr/bin/env python
import tf
import rospy
import tf.transformations as tft
import numpy as np

rospy.init_node('lab24_script')
listener = tf.TransformListener()
rospy.sleep(0.1)

(trans, rot) = (None, None)

while trans is None:
  try:
    (trans, rot) = listener.lookupTransform('base_link', 'gripper_link', rospy.Time(0))
  except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
    pass

sq_mtx = tft.quaternion_matrix([0, 0, 0.38268343, 0.92387953])
st_mtx = tft.translation_matrix([0.6 - 0.1, -0.1, 0.7])

tq_mtx = tft.quaternion_matrix(rot)
tt_mtx = tft.translation_matrix(trans)

rq_mtx = np.dot(tq_mtx, sq_mtx)
rt_mtx = np.dot(tt_mtx, st_mtx)

rt = tft.translation_from_matrix(rt_mtx)
rq = tft.quaternion_from_matrix(rq_mtx)

print rt
print rq
