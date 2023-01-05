#!/usr/bin/env bash
echo "来料检测工具合集"

USER=$(whoami)
TFOLDER="/home/${USER}/catkin_ws/src/augustbot-tools/assemble_tools/src/assemble_tools"
echo '''
输入1,启动Power_sensor检测
输入2,启动IMU检测
输入3,启动倾角仪检测
输入4,启动Lidar检测
'''
read tools_num
if test $tools_num -eq 1
then
    echo "启动Power_sensor检测"
    sh -c "roslaunch assemble_tools e4_power_module_check.launch --screen"
elif test $tools_num -eq 2
then
    echo "启动IMU检测"
    sh -c "roslaunch assemble_tools e3_imu_check.launch --screen"
elif test $tools_num -eq 3
then
    echo "启动倾角仪检测"
    sh -c "roslaunch assemble_tools b10_mk_inclinometer_check.launch --screen"
elif test $tools_num -eq 4
then
    echo "启动Lidar检测"
    sh -c "roslaunch ydlidar_ros_driver lidar_view.launch --screen"
else
    exit 1
fi
