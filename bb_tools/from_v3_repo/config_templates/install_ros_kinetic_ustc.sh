#!/bin/bash
# Apache License 2.0
# Copyright (c) 2017, ROBOTIS CO., LTD.

STARTING_DATE="`date -R`"
echo "[Script starting at $STARTING_DATE]"
echo ""
echo "[Note] Target OS version  >>> Ubuntu 16.04.x (xenial) or Linux Mint 18.x"
echo "[Note] Target ROS version >>> ROS Kinetic Kame"
echo "[Note] Catkin workspace   >>> $HOME/catkin_ws"
echo ""
echo "PRESS [ENTER] TO CONTINUE THE INSTALLATION"
echo "IF YOU WANT TO CANCEL, PRESS [CTRL] + [C]"
read

echo "[Set the target OS, ROS version and name of catkin workspace]"
name_os_version=${name_os_version:="xenial"}
name_ros_version=${name_ros_version:="kinetic"}
name_catkin_workspace=${name_catkin_workspace:="catkin_ws"}

echo "[Update the package lists]"
sudo apt-get update -y
# sudo apt-get upgrade -y

echo "[Install build environment, the chrony, ntpdate and set the ntpdate]"
sudo apt-get install -y chrony ntpdate
sudo ntpdate ntp.ubuntu.com


echo "[Add the ROS repository]"
if [ ! -e /etc/apt/sources.list.d/ros-latest.list ]; then
  sudo sh -c "echo \"deb http://mirrors.ustc.edu.cn/ros/ubuntu ${name_os_version} main\" > /etc/apt/sources.list.d/ros-latest.list"
fi

echo "[Download the ROS keys]"
roskey=`apt-key list | grep "ROS Builder"`
if [ -z "$roskey" ]; then
  sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
fi

echo "[Check the ROS keys]"
roskey=`apt-key list | grep "ROS Builder"`
if [ -n "$roskey" ]; then
  echo "[ROS key exists in the list]"
else
  echo "[Failed to receive the ROS key, aborts the installation]"
  exit 0
fi

echo "[Update the package lists and]"
sudo apt-get update -y
# sudo apt-get upgrade -y

echo "[Install the ros-desktop]"
sudo apt-get install -y ros-$name_ros_version-desktop

echo "[Install essential packages: perception]"
sudo apt-get install -y ros-$name_ros_version-perception

echo "[Install essential packages: navigation]"
sudo apt-get install -y ros-$name_ros_version-navigation

echo "[Initialize rosdep]"
sudo sh -c "rosdep init"
rosdep update

echo "[Environment setup and getting rosinstall]"
source /opt/ros/$name_ros_version/setup.sh
sudo apt-get install -y python-rosinstall python-rosinstall-generator python-wstool build-essential

echo "[Installing vim and git]"
sudo apt-get install -y git
sudo apt-get install -y vim 
sudo apt-get install -y openssh-server 
sudo apt-get install -y python-pip 

echo "[Make the catkin workspace and test the catkin_make]"
sh -c "echo \"source /opt/ros/$name_ros_version/setup.bash\" >> ~/.bashrc"
sh -c "echo \"source ~/$name_catkin_workspace/devel/setup.bash\" >> ~/.bashrc"
source $HOME/.bashrc

mkdir -p $HOME/$name_catkin_workspace/src
cd $HOME/$name_catkin_workspace/src
catkin_init_workspace
cd $HOME/$name_catkin_workspace
catkin_make

echo "[Set the user groups]"
sudo sh -c "usermod -aG dialout $USER"
sudo sh -c "usermod -aG video $USER"

echo "[Set the ROS evironment]"
sh -c "echo \"alias eb='vim ~/.bashrc'\" >> ~/.bashrc"
sh -c "echo \"alias sb='source ~/.bashrc'\" >> ~/.bashrc"
sh -c "echo \"alias gs='git status'\" >> ~/.bashrc"
sh -c "echo \"alias gp='git pull'\" >> ~/.bashrc"
sh -c "echo \"alias rdi='rosdep install --from-path src --ignore-src --rosdistro=kinetic -y'\" >> ~/.bashrc"
sh -c "echo \"alias cw='cd ~/$name_catkin_workspace'\" >> ~/.bashrc"
sh -c "echo \"alias cs='cd ~/$name_catkin_workspace/src'\" >> ~/.bashrc"
sh -c "echo \"alias cm='cd ~/$name_catkin_workspace && catkin_make'\" >> ~/.bashrc"
sh -c "ln -sr `dirname $0`/swmaster.sh ~/swmaster.sh"
sudo sh -c "cp `dirname $0`/50-no-guest.conf /usr/share/lightdm/lightdm.conf.d/50-no-guest.conf"

# sh -c "echo \"export ROS_MASTER_URI=http://localhost:11311\" >> ~/.bashrc"
# sh -c "echo \"export ROS_HOSTNAME=localhost\" >> ~/.bashrc"

source $HOME/.bashrc

echo "[Complete!!!]"
echo "[Script starting at $STARTING_DATE]"
echo "[Script finished at `date -R`]"
exit 0
