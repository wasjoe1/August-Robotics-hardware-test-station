#!/bin/bash
source /home/augbooth/swmaster.sh;
stdbuf -o L roslaunch --wait tools dtu_server_for_gs_aging.launch;
