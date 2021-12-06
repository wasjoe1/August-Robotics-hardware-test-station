#!/bin/bash

CATKIN_WS=/home/augbooth/catkin_ws

echo "Doing catkin clean now...";
catkin clean -y -w $CATKIN_WS;

echo "Cleaning done, start catkin build now..."
catkin build -c -s --unbuilt -w $CATKIN_WS;

#FIXME: This is a temperate solution "只build一次，有时候simulator 会出现问题。ROS 会告诉我少了一些modules/libraries"
echo "Start catkin build second times..."
catkin build -c -s --unbuilt -w $CATKIN_WS;