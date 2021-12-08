#!/bin/bash
source /opt/ros/melodic/setup.bash;

echo "" >> /home/augbooth/.bashrc;
echo "export TC_HOST_IP=127.0.0.1 # this is your large PC IP" >> /home/augbooth/.bashrc;
echo "export GS_LAN_PORT=lo # this is GS pc LAN port" >> /home/augbooth/.bashrc;
echo "export PTY_DTU=1 # when using PTY as DTU." >> /home/augbooth/.bashrc;
echo "source /home/augbooth/catkin_ws/devel/setup.bash" >> /home/augbooth/.bashrc;

while true;
do
OUTPUT=$(curl -s http://localhost:11311)
if [ -z "$OUTPUT" ]; then
    echo "Waiting for roscore container to finish launching";
else
    echo "roscore container has started, launch backoffice now";
    break
fi
sleep 1
done

source /home/augbooth/catkin_ws/devel/setup.bash;

# As sourcing .bashrc won't work within a bash script, using this way to work around
source ./env.bash;

cd /home/augbooth/catkin_ws/src/boothbot/backoffice;
./manage.py migrate;
./manage.py runserver 0.0.0.0:8000
