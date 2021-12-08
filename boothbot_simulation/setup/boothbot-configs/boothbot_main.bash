#!/bin/bash -i
source /opt/ros/melodic/setup.bash;

echo "" >> /home/augbooth/.bashrc;
echo "export TC_HOST_IP=127.0.0.1 # this is your large PC IP" >> /home/augbooth/.bashrc;
echo "export GS_LAN_PORT=lo # this is GS pc LAN port" >> /home/augbooth/.bashrc;
echo "export PTY_DTU=1 # when using PTY as DTU." >> /home/augbooth/.bashrc;
echo "source /home/augbooth/catkin_ws/devel/setup.bash" >> /home/augbooth/.bashrc;

sudo chown augbooth:augbooth /dev/dtu

while true;
do
OUTPUT=$(curl -s http://localhost:8000/accounts/login/?next=/)
if [ -z "$OUTPUT" ]; then
    echo "Waiting for backoffice";
else
    echo "backoffice is started, launch boothbot_main now";
    break
fi
sleep 1
done

cd /home/augbooth/catkin_ws/src/boothbot/backoffice;
pip install -e .

source /home/augbooth/catkin_ws/devel/setup.bash;

# As sourcing .bashrc won't work within a bash script, using this way to work around
source /home/augbooth/catkin_ws/src/augustbot-tools/boothbot_simulation/setup/boothbot-configs/env.bash;

# This two export need to exec after 'source ./env.bash;'
echo "export GIT_HEAD=${GIT_HEAD}" >> /home/augbooth/.bashrc;
echo "export TODAY=`date +%y-%m-%d` # This is the current branch name" >> /home/augbooth/.bashrc;

echo "Now launch boothbot_main"

stdbuf -o L roslaunch --wait boothbot_main boothbot_main.launch fake_cb:=true using_dtu:=false fake_chassis_usb:=true fixed_lan_ip:=true silent_imu_check:=true

tail -f /dev/null