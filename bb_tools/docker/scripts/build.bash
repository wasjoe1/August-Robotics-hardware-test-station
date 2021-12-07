#!/bin/bash

CATKIN_WS=/home/augbooth/catkin_ws

echo "Doing catkin clean now...";
catkin clean -y -w $CATKIN_WS;

echo "Cleaning done, start catkin build now..."
catkin build -c -s --unbuilt -w $CATKIN_WS;