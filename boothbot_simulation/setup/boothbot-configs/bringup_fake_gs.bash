#!/bin/bash
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
    echo "backoffice is started, launch fake_gs now";
    break
fi
sleep 1
done

source /home/augbooth/catkin_ws/devel/setup.bash;
tail -f /dev/null