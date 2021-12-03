#!/usr/bin/env bash

# Make sure roscore is running

CURRENTUSER=$(whoami)

CMD="roslaunch guiding_beacon camera_beacon.launch no_tracker:=True"
SHOWCMD="python /home/$CURRENTUSER/catkin_ws/src/boothbot/boothbot_perception/test/track_on_device_sharpness.py /dev/camera_short 5 -c green --state_dict /home/$CURRENTUSER/catkin_ws/src/boothbot/boothbot_perception/scripts/boothbot_perception/models/linear_gray.p"
FIXCMD="rostopic pub -1 /cb_0/cmdword std_msgs/String enable"
FREECMD="rostopic pub -1 /cb_0/cmdword std_msgs/String disable"
LASERCMD="rostopic pub -1 /cb_0/cmdword std_msgs/String laseron"

tmux new-session -d -s bb_session -n simple_control
tmux new-window -t bb_session -n simple_control_pub
eval `tmux send-keys -t bb_session:simple_control "${CMD}" C-m`
gnome-terminal -e "bash -ic '${SHOWCMD}; bash'"
gnome-terminal -e "tmux a -t bb_session:simple_control"

echo "will wait for 15 seconds for launching script"
sleep 15
echo "Running..."
echo "取消使能 'n', 使能 'r', 激光 'l', 退出 'e':"

while true
do
read -rsn1 input
if [ "$input" = "n" ]; then
    eval `tmux send-keys -t bb_session:simple_control_pub "${FREECMD}" C-m`
elif [ "$input" = "r" ]; then
    eval `tmux send-keys -t bb_session:simple_control_pub "${FIXCMD}" C-m`
elif [ "$input" = "l" ]; then
    eval `tmux send-keys -t bb_session:simple_control_pub "${LASERCMD}" C-m`
elif [ "$input" = "e" ]; then
    break
else
    echo "Wrong input!"
fi
done

tmux kill-session -t bb_session
