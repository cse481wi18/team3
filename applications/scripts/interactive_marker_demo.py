#!/usr/bin/env python
import rospy
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from visualization_msgs.msg import Marker
from map_annotator.msg import UserAction

# Make the node that controls the marker server
rospy.init_node("interactive_marker_test")

# Make the marker server itself
server = InteractiveMarkerServer("simple_marker")

# Make a single interactive marker
int_marker = InteractiveMarker()
int_marker.header.frame_id = "map"
int_marker.name = "my_marker"
int_marker.description = "Click to Move to Marker"
int_marker.pose.position.x = 1
int_marker.pose.orientation.w = 1

# Create that actual object that the interactive marker will correspond to
box_marker = Marker()
box_marker.type = Marker.CUBE
box_marker.pose.orientation.w = 1
box_marker.scale.x = 0.45
box_marker.scale.y = 0.45
box_marker.scale.z = 0.45
box_marker.color.r = 0.0
box_marker.color.g = 0.5
box_marker.color.b = 0.5
box_marker.color.a = 1.0

# Create the controller to associate with the object
button_control = InteractiveMarkerControl()
button_control.interaction_mode = InteractiveMarkerControl.BUTTON
button_control.always_visible = True
# Associate the object with the control
button_control.markers.append(box_marker)
# Associate the control with the interactive marker
int_marker.controls.append(button_control)

pub = rospy.Publisher("map_annotator/user_actions", UserAction, queue_size=10)


# Callback to handle interactivity
def handle_viz_input(input):
    rospy.logerr("Event type was " + str(input.event_type))
    if (input.event_type == InteractiveMarkerFeedback.BUTTON_CLICK):
        pub.publish(UserAction(command=UserAction.GOTO, name="demo")) 
        rospy.loginfo(input.marker_name + ' was clicked.')

# Add the interactive marker to the server
server.insert(int_marker, handle_viz_input)
server.applyChanges()   

rospy.spin()
