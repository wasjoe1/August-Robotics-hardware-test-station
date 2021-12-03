#!/usr/bin/env bash
# -*- coding: utf-8 -*-

# Make sure roscore is running

CMD="roslaunch guiding_beacon camera_beacon.launch"
RECORD="rosrun guiding_beacon levelness_tuning_cb.py _is_cb:=True"
NEWCMD="rostopic pub -1 /cb_0/cmdword std_msgs/String disable"
RUNCMD="rostopic pub -1 /cb_0/cmdword std_msgs/String enable"
CALICMD="rostopic pub -1 /cb_0/cmdword std_msgs/String disable"

WNAME="levelness_tuning"
PWNAME="levelness_tuning_pub"

tmux new-session -d -s bb_session -n ${WNAME}
tmux new-window -t bb_session -n ${PWNAME}
eval `tmux send-keys -t bb_session:${WNAME} "${CMD}" C-m`
gnome-terminal -e "bash -ic '${RECORD}; bash'"
gnome-terminal -e "tmux a -t bb_session:${WNAME}"

echo "Wait 10 seconds for Running"
sleep 10
echo "Running..."
echo "取消使能 'n', 追踪 'r', 开始 's', 退出 'e':"

while true
do
read -rsn1 input
if [ "$input" = "n" ]; then
    eval `tmux send-keys -t bb_session:${PWNAME} "${NEWCMD}" C-m`
elif [ "$input" = "r" ]; then
    eval `tmux send-keys -t bb_session:${PWNAME} "${RUNCMD}" C-m`
elif [ "$input" = "s" ]; then
    eval `tmux send-keys -t bb_session:${PWNAME} "${CALICMD}" C-m`
elif [ "$input" = "e" ]; then
    break
else
    echo "Wrong input!"
fi
done

tmux kill-session -t bb_session
