#!/usr/bin/env python

import tf
import rospy

if __name__ == '__main__':
  rospy.init_node('ee_pose_demo')
  listener = tf.TransformListener()
  rospy.sleep(0.1)
  rate = rospy.Rate(1.0)

  while not rospy.is_shutdown():
    try:
      (trans, rot) = listener.lookupTransform('base_link', 'gripper_link', rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
      continue
    
    if trans is None or rot is None:
      print '"base_link" passed to lookupTransform argument target_frame does not exist.'
    else:
      print str(tuple(trans)) + ' ' + str(tuple(rot))
    
    try:
      rate.sleep()
    except (rospy.exceptions.ROSInterruptException):
      break
