#!/usr/bin/env bash
echo "sonar工具合集"

USER=$(whoami)
TFOLDER="/home/${USER}/catkin_ws/src/augustbot-tools/assemble_tools/D_声呐"
echo '''
输入1,启动sonar地址写入
输入2,启动sonar位置检测测试
'''
read tools_num
if test $tools_num -eq 1
then
    echo "sonar地址写入"
    sh -c "python3 ${TFOLDER}/DYP_sonar_unit_writer.py"
elif test $tools_num -eq 2
then
    echo "sonar位置检测测试"
    sh -c "python3 ${TFOLDER}/DYP_sonar_checker.py"
else
    exit 1
fi
