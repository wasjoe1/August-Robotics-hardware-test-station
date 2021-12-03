#!/usr/bin/env bash
# -*- coding: utf-8 -*-

# Make sure roscore is running

CMD="rosrun guiding_beacon two_cameras_angle_offset.py _is_cb:=True"
RUNCMD="rostopic pub -1 /camera_angle_offset/cmdword std_msgs/String target"

tmux new-session -d -s bb_session -n two_camera_angle_offset
tmux new-window -t bb_session -n two_camera_angle_offset_pub
eval `tmux send-keys -t bb_session:two_camera_angle_offset "${CMD}" C-m`
gnome-terminal -e "tmux a -t bb_session:two_camera_angle_offset"

echo "Wait 10 seconds for Running"
sleep 10
echo "Running..."
echo "追踪 'r', 退出 'e':"

while true
do
read -rsn1 input
if [ "$input" = "r" ]; then
    eval `tmux send-keys -t bb_session:two_camera_angle_offset_pub "${RUNCMD}" C-m`
elif [ "$input" = "e" ]; then
    break
else
    echo "Wrong input!"
fi
done

tmux kill-session -t bb_session
