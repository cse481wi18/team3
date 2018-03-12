#!/bin/bash
SESSION=demo

# window 1
tmux -2 new-session -d -s $SESSION
tmux send-keys "roslaunch applications nav_rviz.launch is_sim:=false" C-m
sleep 1
tmux new-window -t $SESSION:1 -n $SESSION
tmux split-window -h
tmux split-window -v
tmux select-pane -t 0
tmux send-keys "rosrun applications sound_demo.py" C-m
tmux select-pane -t 1
tmux send-keys "rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 baud:=57600" C-m
tmux select-pane -t 2
tmux send-keys "rostopic pub /navigation_controller/max_speed std_msgs/Float32 0.5" C-m
tmux send-keys "rostopic pub /navigation_controller/max_rotation std_msgs/Float32 0.75" C-m

# window 2
tmux new-window -t $SESSION:2 -n $SESSION
tmux split-window -h
tmux select-pane -t 0
tmux send-keys "roslaunch fetch_api move_group.launch" C-m
tmux select-pane -t 1
tmux send-keys "roslaunch fetch_api ar_desktop.launch" C-m

# window 3
tmux new-window -t $SESSION:3 -n $SESSION
tmux split-window -h
tmux select-pane -t 0
# not needed
#tmux send-keys "python ~/frontend/robotics-ios/robotics_server.py 5000" C-m
sleep 0.5
tmux select-pane -t 1
tmux send-keys "python ~/robotics-ios/server_ros_bridge.py attu3.cs.washington.edu 5000" C-m

#not here
tmux -2 attach-session -t $SESSION
