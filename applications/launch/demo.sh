#!/bin/bash
SESSION=demo

set_pane_title() {
	printf '\033]2;%s\033\\' $@
}

# window 1
tmux -2 new-session -d -s $SESSION
tmux send-keys "roslaunch applications nav_rviz.launch is_sim:=false" C-m
sleep 1
tmux new-window -t $SESSION:1 -n "Reliable ROS Nodes"
tmux split-window -h
tmux split-window -v
tmux select-pane -t 0
set_pane_title "Sound Demo"
tmux send-keys "rosrun applications sound_demo.py" C-m
tmux select-pane -t 1
set_pane_title "Rosserial"
tmux send-keys "rosrun applications reset_serial_port.py && rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=57600" C-m
tmux select-pane -t 2
set_pane_title "Max Speed Publisher"
tmux send-keys "rostopic pub /navigation_controller/max_speed std_msgs/Float32 0.5" C-m
set_pane_title "Max Rotation Publisher"
tmux send-keys "rostopic pub /navigation_controller/max_rotation std_msgs/Float32 0.75" C-m

# window 2
tmux new-window -t $SESSION:2 -n "Mission Control"
tmux split-window -h
tmux split-window -v
tmux select-pane -t 0
set_pane_title "Move Group"
tmux send-keys "roslaunch fetch_api move_group.launch" C-m
tmux select-pane -t 1
set_pane_title "Ar Tracker"
tmux send-keys "roslaunch fetch_api ar_desktop.launch" C-m
tmux select-pane -t 2
set_pane_title "Server Bridge"
tmux send-keys "python ~/robotics-ios/server_ros_bridge.py attu3.cs.washington.edu 5000" C-m

#not here
tmux -2 attach-session -t $SESSION
