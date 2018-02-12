#!/usr/bin/env python
import tf
import tf.transformations as tft
import math
import numpy as np
import rospy
from copy import deepcopy
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback, MenuEntry
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Quaternion, Pose, PoseStamped
from std_msgs.msg import ColorRGBA

from fetch_api import Arm, Gripper

def lookup_transform(a, b):
  listener = tf.TransformListener()
  rospy.sleep(0.1)
  pose = None
  while pose is None:
    try:
      pose = listener.lookupTransform(a, b, rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
      pass
  return Pose(Point(*pose[0]), Quaternion(*pose[1]))

def create_marker(mesh_path, x=0.166):
  marker = Marker()
  marker.type = Marker.MESH_RESOURCE
  marker.mesh_resource = mesh_path
  marker.pose.orientation.w = 1
  marker.scale.x = 1
  marker.scale.y = 1
  marker.scale.z = 1
  marker.color = ColorRGBA(0, 1, 0, 1)
  marker.pose.position.x = x
  return marker

def make_6dof_controls():
  controls = list()
  control = InteractiveMarkerControl()
  control.orientation.w = 1;
  control.orientation.x = 1;
  control.orientation.y = 0;
  control.orientation.z = 0;
  control.name = "rotate_x";
  control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS;
  controls.append(control);
  control = deepcopy(control)
  control.name = "move_x";
  control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS;
  controls.append(control);

  control = InteractiveMarkerControl()
  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.name = "rotate_z";
  control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS;
  controls.append(control);
  control = deepcopy(control)
  control.name = "move_z";
  control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS;
  controls.append(control);

  control = InteractiveMarkerControl()
  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 1;
  control.name = "rotate_y";
  control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS;
  controls.append(control);
  control = deepcopy(control)
  control.name = "move_y";
  control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS;
  controls.append(control); 
  return controls


class GripperTeleop(object):
    def __init__(self, arm, gripper, im_server):
        self._arm = arm
        self._gripper = gripper
        self._im_server = im_server
        self._im = None

    def _create_interactive_marker(self, pose):
        #TODO: implement this
        # creates an interactive marker for the gripper controller
        # * shaped like robot's gripper
        # * 6dof-movable
        # * clickable menu
        # returns it
        rospy.logerr( "create interactive marker!")
        im = InteractiveMarker()
        im.header.frame_id = "base_link"
        im.name = "gripper_control"
        im.description = "Gripper Control"
        im.pose = pose
        gripper  = create_marker("package://fetch_description/meshes/gripper_link.dae")
        finger_l = create_marker("package://fetch_description/meshes/l_gripper_finger_link.STL")
        finger_l.pose.position.y = -0.06
        finger_r = create_marker("package://fetch_description/meshes/r_gripper_finger_link.STL")
        finger_r.pose.position.y = +0.06
        control = InteractiveMarkerControl()
        control.always_visible = True
        control.markers.append(gripper)
        control.markers.append(finger_l)
        control.markers.append(finger_r)
        # menu items!
        move_gripper = MenuEntry()
        move_gripper.id = 1
        move_gripper.parent_id = 0
        move_gripper.title = "Move gripper here"
        move_gripper.command = "FEEDBACK"
        im.menu_entries.append(move_gripper)
        open_gripper = MenuEntry()
        open_gripper.id = 2
        open_gripper.parent_id = 0
        open_gripper.title = "Open gripper"
        open_gripper.command = "FEEDBACK"
        im.menu_entries.append(open_gripper)
        close_gripper = MenuEntry()
        close_gripper.id = 3
        close_gripper.parent_id = 0
        close_gripper.title = "Close gripper"
        close_gripper.command = "FEEDBACK"
        im.menu_entries.append(close_gripper)
        control.interaction_mode = InteractiveMarkerControl.MENU

        im.controls.append(control)
        # controls for 6dofs
        controls = make_6dof_controls()
        im.controls.extend(controls)
        im.scale = 0.3
        return im
    
    def _set_im(self, im):
        self._im_server.insert(im, feedback_cb=self.handle_feedback)
        self._im_server.applyChanges()

    def start(self):
        rospy.logerr("start")
        im = self._create_interactive_marker(lookup_transform("base_link", "gripper_link"))
        self._set_im(im)
        rospy.logerr("end start")
    
    def _open(self):
      self._gripper.open()

    def _close(self):
      self._gripper.close()

    def _move(self, pose_stamped):
      self._arm.move_to_pose(pose_stamped)

    def _check_pose(self, pose_stamped):
      return self._arm.compute_ik(pose_stamped)

    def _redraw_with_color(self, reachable, pose_stamped):
      rospy.logerr("Reachable" + str(reachable))
      color = ColorRGBA(0, 1, 0, 1) if reachable else ColorRGBA(1, 0, 0, 1)
      im = self._im_server.get("gripper_control")
      for marker in im.controls[0].markers:
        marker.color = color
      self._set_im(im)
    
    def handle_feedback(self, f):
        rospy.logerr( "handling feedback!")
        im = self._im_server.get("gripper_control")
        pose_stamped = PoseStamped(pose=im.pose)
        pose_stamped.header.frame_id = "/base_link"
        pose_stamped.header.stamp = rospy.Time.now()
        if (f.event_type == f.MENU_SELECT):
          fid = f.menu_entry_id
          if (fid == 1):
            self._move(pose_stamped)
          elif (fid == 2):
            self._open()
          elif (fid == 3):
            self._close()
        elif (f.event_type == f.POSE_UPDATE):
          self._redraw_with_color(self._check_pose(pose_stamped), pose_stamped)
          #pass
        # handle user action on the interactive marker
        # if it is a menu press, do the corresponding action
        #   (go to, open, close)
        #   using the moveit ik server (see those demos)
        # if it is a movement
        #   update the color of the markers depending on reachability
        #   using the moveit ik server

# TODO: TRANSFOMRATION MATRIX.... PLEASE VERIFY
grip_in_obj = np.array([
  [1, 0, 0, 0],
  [0, 1, 0, 0],
  [0, 0, 1, 0],
  [0, 0, 0, 1]
])

pre_in_obj = np.array([
  [1, 0, 0, -0.1],
  [0, 1, 0, 0],
  [0, 0, 1, 0],
  [0, 0, 0, 1]
])
lift_in_obj = np.array([
  [1, 0, 0, 0],
  [0, 1, 0, 0],
  [0, 0, 1, 0.2],
  [0, 0, 0, 1]
])
# TODO: Write this class (it should mostly be similar to the above class, so finish that one first)
class AutoPickTeleop(object):
    def __init__(self, arm, gripper, im_server):
        self._arm = arm
        self._gripper = gripper
        self._im_server = im_server

    def create_gripper_marker_control(self, dx=0, dz=0):
        gripper  = create_marker("package://fetch_description/meshes/gripper_link.dae")
        gripper.pose.position.x -= dx
        gripper.pose.position.z += dz
        finger_l = create_marker("package://fetch_description/meshes/l_gripper_finger_link.STL")
        finger_l.pose.position.y = -0.06
        finger_l.pose.position.x -= dx
        finger_l.pose.position.z += dz
        finger_r = create_marker("package://fetch_description/meshes/r_gripper_finger_link.STL")
        finger_r.pose.position.x -= dx
        finger_r.pose.position.y = +0.06
        finger_r.pose.position.z += dz
        control = InteractiveMarkerControl()
        control.markers.append(gripper)
        control.markers.append(finger_l)
        control.markers.append(finger_r)
        control.always_visible = True
        return control

    def _set_im(self, im):
        self._im_server.insert(im, feedback_cb=self.handle_feedback)
        self._im_server.applyChanges()

    def _create_interactive_marker(self, pose):
        # creates an interactive marker for the gripper controller
        # * shaped like robot's gripper
        # * 6dof-movable
        # * clickable menu
        # returns it
        ##### SETUP INTERACTIVE MARKER
        rospy.logerr( "create interactive marker!")
        im = InteractiveMarker()
        im.header.frame_id = "base_link"
        im.name = "gripper_control"
        im.description = "Gripper Control"
        im.pose = pose
        # TODO: setup box marker
        box = Marker()
        box.type = Marker.CUBE
        box.pose.orientation.w = 1
        box.scale.x = 0.05
        box.scale.y = 0.05
        box.scale.z = 0.05
        box.color = ColorRGBA(1, 1, 0, 1)
        box_control = InteractiveMarkerControl()
        box_control.markers.append(box)
        im.controls.append(box_control)
        # TODO: setup grasp marker
        control = self.create_gripper_marker_control()

        # TODO: setup lift marker
        lift_control = self.create_gripper_marker_control(dz=0.2)

        ##### TODO: SETTING UP PRE-GRASP MARKER
        pre_control = self.create_gripper_marker_control(dx=0.1)
        
        # add controller for grippers in to InteractiveMarker
        im.controls.append(control)
        im.controls.append(lift_control)
        im.controls.append(pre_control)

        # controls for 6dofs
        controls = make_6dof_controls()
        im.controls.extend(controls)

        # menu items!
        move_gripper = MenuEntry()
        move_gripper.id = 1
        move_gripper.parent_id = 0
        move_gripper.title = "Pick from front"
        move_gripper.command = "FEEDBACK"
        im.menu_entries.append(move_gripper)
        open_gripper = MenuEntry()
        open_gripper.id = 2
        open_gripper.parent_id = 0
        open_gripper.title = "Open gripper"
        open_gripper.command = "FEEDBACK"
        im.menu_entries.append(open_gripper)
        control.interaction_mode = InteractiveMarkerControl.MENU

        im.scale = 0.3
        return im
 

    def start(self):
        # obj_im = InteractiveMarker() ...
        im = self._create_interactive_marker(lookup_transform("base_link", "gripper_link"))
        self._set_im(im)
        pose_stamped = PoseStamped(pose=im.pose)
        pose_stamped.header.frame_id = "/base_link"
        pose_stamped.header.stamp = rospy.Time.now()

        for i in range(1, 4):
          self._redraw_with_color(self._check_pose(pose_stamped, i), pose_stamped, index=i)


    def _open(self):
      self._gripper.open()

    def _close(self):
      self._gripper.close()

    def _move(self, pose_stamped):
      ps = self.calculate_pose_stamp(pose_stamped, pre_in_obj)
      self._arm.move_to_pose(ps)
      ps = self.calculate_pose_stamp(pose_stamped, grip_in_obj)
      self._arm.move_to_pose(ps)
      self._close()
      ps = self.calculate_pose_stamp(pose_stamped, lift_in_obj)
      self._arm.move_to_pose(ps)

    def calculate_pose_stamp(self, pose_stamped, grip_in_obj):
      # pose_stamed is the new updated position of object
      pos = pose_stamped.pose.position
      ori = pose_stamped.pose.orientation
      obj_in_base = np.dot(tft.translation_matrix([pos.x, pos.y, pos.z]), tft.quaternion_matrix([ori.x, ori.y, ori.z, ori.w]))
      gripper_in_base = np.dot(obj_in_base, grip_in_obj)
      pose = Pose(Point(*tft.translation_from_matrix(gripper_in_base)), Quaternion(*tft.quaternion_from_matrix(gripper_in_base)))
      pose_stamped = PoseStamped()
      pose_stamped.header.frame_id = "/base_link"
      pose_stamped.header.stamp = rospy.Time.now()
      pose_stamped.pose = pose
      return pose_stamped


    def _check_pose(self, pose_stamped, i=1):
      # pose_stamed is the new updated position of object
      if i == 1:
        pose_stamped = self.calculate_pose_stamp(pose_stamped, grip_in_obj)
      if i == 2:
        pose_stamped = self.calculate_pose_stamp(pose_stamped, lift_in_obj)
      elif i == 3:
        pose_stamped = self.calculate_pose_stamp(pose_stamped, pre_in_obj)
      return self._arm.compute_ik(pose_stamped)

    def _redraw_with_color(self, reachable, pose_stamped, index=0):
      rospy.logerr("Reachable" + str(reachable))
      color = ColorRGBA(0, 1, 0, 1) if reachable else ColorRGBA(1, 0, 0, 1)
      im = self._im_server.get("gripper_control")
      for marker in im.controls[index].markers:
        marker.color = color
      self._set_im(im)
 
    def handle_feedback(self, f):
        rospy.logerr( "handling feedback!")
        im = self._im_server.get("gripper_control")
        pose_stamped = PoseStamped(pose=im.pose)
        pose_stamped.header.frame_id = "/base_link"
        pose_stamped.header.stamp = rospy.Time.now()
        if (f.event_type == f.MENU_SELECT):
          fid = f.menu_entry_id
          # pick from front
          if (fid == 1):
            self._move(pose_stamped)
          # open grip
          elif (fid == 2):
            self._open()
        elif (f.event_type == f.POSE_UPDATE):
          for i in range(1, 4):
            self._redraw_with_color(self._check_pose(pose_stamped, i), pose_stamped, index=i)


def main():
    rospy.init_node("gripper_teleop")
    #...
    im_server = InteractiveMarkerServer('gripper_im_server', q_size=2)
    # auto_pick_im_server = InteractiveMarkerServer('auto_pick_im_server', q_size=2)
    teleop = GripperTeleop(Arm(), Gripper(), im_server)
    # auto_pick = AutoPickTeleop(Arm(), Gripper(), auto_pick_im_server)
    teleop.start()
    # auto_pick.start()
    rospy.spin()

if __name__ == "__main__":
  main()
