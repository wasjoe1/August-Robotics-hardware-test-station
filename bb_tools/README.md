# README.md

This repo is a testground for debug tools (BoothBot-related).
As these are all python scripts, no dependency is listed in `package.xml` 
or `CMakeList.txt`. You have need a working `catkin_ws` to use this tools.

## TL;DR

```bash
# Web terminal/docker
docker pull wettyoss/wetty
docker run --rm --detach --name wetty_pod -p 3000:3000 wettyoss/wetty --ssh-host=172.17.0.1 --ssh-user=augbooth\
--command="bash --rcfile <(echo '. ~/.bashrc; rosrun bb_debug dash.py; exit')"

# ROS/python
cd ~/catkin_ws/src && git clone git@gitee.com:august-robotics/bb-tools.git
cd bb-tools && python3 -m pip install -r requirements.txt
cd ~/catkin_ws && catkin_make # or catkin build
rosrun rosrun bb_debug dash.py 
```