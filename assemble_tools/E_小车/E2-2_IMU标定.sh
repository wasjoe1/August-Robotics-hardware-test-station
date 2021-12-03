#!/usr/bin/env bash

# Make sure roscore is running
SETUP="SETUP"
CALI_ARG="CALIBRATE"
SAVE_ARG="SAVE"


CMD="roslaunch ../launch/base_production.launch"
RECORD="rostopic echo -c /drivers/chassis/imu"
SETUPCMD="rosservice call /drivers/chassis/srv_cmd '{command: "IMU", parameter: ${SETUP}}'"
CALICMD="rosservice call /drivers/chassis/srv_cmd '{command: "IMU", parameter: ${CALI_ARG}}'"
SAVECMD="rosservice call /drivers/chassis/srv_cmd '{command: "IMU", parameter: ${SAVE_ARG}}'"

WNAME="imu_incli_tuning"
PWNAME="imu_incli_tuning_pub"

tmux new-session -d -s bb_session -n ${WNAME}
tmux new-window -t bb_session -n ${PWNAME}
eval `tmux send-keys -t bb_session:${WNAME} "${CMD}" C-m`
gnome-terminal -- tmux a -t bb_session:${WNAME}

read -p "Enter the robot number: " filed_name
echo $filed_name
timenow=$(date "+%Y-%m-%d-%H:%M:%S")
echo ${timenow}
if  [ ! -e "../Y_Doc/$filed_name.yaml" ];then
  cp -i "../Y_Doc/default.yaml" "../Y_Doc/$filed_name.yaml"
  echo 'create the yaml file '
else
  echo 'already have the local yaml file'
fi
echo "Wait 3 seconds for Running"
sleep 3
eval `tmux send-keys -t bb_session:${PWNAME} "${SETUP}" C-m`
gnome-terminal -- bash -ic "${RECORD};exec bash"
PID=$(pgrep bash -a |grep exec)
PID=($PID)
PID=${PID[0]}
sleep 2
echo "Running..."
echo "校准 'c', 退出 'e':"
echo "Calibration:'c',Exit:'e'"
echo "请确保x,y值在小数点后3个位都为零再保存结果, 否则, 请再按c键!"
echo "Please make sure the accuracy of ium x,y is smaller than 0.000*,then press 'c'! "



while true
do
read -rsn1 input
if [ "$input" = "c" ]; then
    eval `tmux send-keys -t bb_session:${PWNAME} "${CALICMD}" C-m`
    echo "Calibrating ..."
    sleep 6
    eval `tmux send-keys -t bb_session:${PWNAME} "${SAVECMD}" C-m`
    echo "Saving ..."
    sleep 3
    eval `tmux send-keys -t bb_session:${PWNAME} "${SAVECMD}" C-m`
    sleep 3
    echo "保存成功!"
    echo "Save successed!"
    sed -i "2s/false/true/g" "../Y_Doc/$filed_name.yaml"
    sed -i "1c# $timenow" "../Y_Doc/$filed_name.yaml"
elif [ "$input" = "e" ]; then
    break
else
    echo "Wrong input!"
fi
done

tmux kill-session -t bb_session
kill -HUP $PID
