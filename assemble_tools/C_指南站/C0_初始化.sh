#!/usr/bin/env bash
# -*- coding: utf-8 -*-

exec < /dev/tty

# check if bring_up_hardware has already auto started 
is_bring_up=$(pgrep ros -a |grep bringup_hardware.launch)
if [ ! $is_bring_up ];then
    CBLAUNCH="roslaunch ../launch/cb_production.launch"
else
    echo "bring_up_hard have auto start"
    CBLAUNCH="roslaunch ../launch/cb_production.launch is_bring_up:=true"
fi
# CMD
FIXCMD="rostopic pub -1 /cb_0/cmdword std_msgs/String enable"
FREECMD="rostopic pub -1 /cb_0/cmdword std_msgs/String disable"
LASERCMD="rostopic pub -1 /cb_0/cmdword std_msgs/String laseron"
ECHOCMD="rostopic echo -c /cb_0/hardware_status/joint"
CLOSESONAR="rosservice call /drivers/chassis/srv_cmd '{\"command\": \"SIDE_SONAR\",\"parameter\":\"OFF\"}'"

read -p 'Enter device name(e.g. LB04-GM03LB03-IS02-0009): ' device_name
read -p "Enter the robot number: " filed_name
timenow=$(date "+%Y-%m-%d-%H:%M:%S")
echo ${timenow}

# check if already have the yaml file
if  [ ! -e "../Y_Doc/$filed_name.yaml" ];then
  cp -i "../Y_Doc/default.yaml" "../Y_Doc/$filed_name.yaml"
  echo 'create the yaml file '
else
  echo 'already have the local yaml file'
fi

folder_path=~/catkin_ws/src/boothbot/common/scripts/common/device_settings/$device_name
file_path=~/catkin_ws/src/boothbot/common/scripts/common/device_settings/$device_name/device_settings.yaml
name_path=~/catkin_ws/src/boothbot/common/scripts/common/device_name.yaml

# check if already have the device yaml
if [ -d "$folder_path" ]; then
    if [ -d "/tmp/$device_name" ]; then
	echo 'delect the device setting to temp'
        rm -r /tmp/$device_name
    fi
    read -p "Moving old settings to /tmp/ folder[y/n]" start_new
    if [ "$start_new" = "y" ]; then
        mv $folder_path /tmp/
        echo "Moving Done!"
        echo "Now copying new settings and setup device_name"
        cp -r CB_default_settings $folder_path
        echo "name: $device_name" > $name_path
    fi
else
   echo "Now copying new settings and setup device_name"
   cp -r CB_default_settings $folder_path
   echo "name: $device_name" > $name_path
fi

echo "Launching CB Now.."
tmux new-session -d -s bb_session -n initial_CB
tmux new-window -t bb_session -n initial_CB_pub
eval `tmux send-keys -t bb_session:initial_CB "${CBLAUNCH}" C-m`
gnome-terminal -- tmux a -t bb_session:initial_CB

echo "will wait for 15 seconds for launching script"
sleep 15
echo "Running..."
gnome-terminal -- bash -ic "${ECHOCMD};exec bash"
echo "取消使能 'n', 使能 'r', save: 's', 退出 'e':"
echo "disable:'n',enable:'r',save:'s',exit:'e'"
eval `tmux send-keys -t bb_session:initial_CB_pub "${CLOSESONAR}" C-m`
sleep 0.01
eval `tmux send-keys -t bb_session:initial_CB_pub "${FREECMD}" C-m`
PID=$(pgrep bash -a |grep exec)
PID=($PID)
PID=${PID[0]}
echo "SONAR have been OFF"
while true
do
read -rsn1 input
if [ "$input" = "n" ]; then
    eval `tmux send-keys -t bb_session:initial_CB_pub "${FREECMD}" C-m`
elif [ "$input" = "r" ]; then
    eval `tmux send-keys -t bb_session:initial_CB_pub "${FIXCMD}" C-m`
elif [ "$input" = "s" ]; then
    top=$(rostopic echo -n 1 /cb_0/hardware_status/joint |grep effort)
    echo $top
    sed -i "4c$top" "../Y_Doc/$filed_name.yaml"
    sed -i "3c# $timenow" "../Y_Doc/$filed_name.yaml"
    top=$(echo ${top} |grep -oP '\d*\.\d+')
    top=($top)
    hor_zero=${top[1]}
    ver_zero=${top[2]}
    if [ "$start_new" = "y" ]; then
       sed -i "27s/0/${hor_zero}/g" $file_path
       sed -i "35s/0/${ver_zero}/g" $file_path
       echo 'change the device setting'
    fi
    echo 'save succeed !'
elif [ "$input" = "e" ]; then
    break
else
    echo "Wrong input!"
fi
done
if [ '$start_new' = 'y' ]; then
   sed -i "27s/0/${hor_zero}/g" $file_path
   sed -i "35s/0/${ver_zero}/g" $file_path
fi
tmux kill-session -t bb_session
kill -HUP $PID

#read -p "horizontoal zero_offset:" hor_zero
#read -p "vertical zero_offset:" ver_zero
#sed -i "27s/0/${hor_zero}/g" $file_path
#sed -i "35s/0/${ver_zero}/g" $file_path
