# Setups for your own pc
export _SETUP_DONE="Not done!"

export BOOTHBOT_IMAGE="augustrobotics/lionel-x86:v3.1"
export DEVICE_CONFIG_PATH="/home/pat/catkin_ws/local"
export ROS_HOME="$DEVICE_CONFIG_PATH/roslogs"
export BOOTHBOT_BACKOFFICE_LOG_PATH="$DEVICE_CONFIG_PATH/backoffice"
export CATKIN_WS_PATH="/home/pat/catkin_ws/docker/catkin_ws"
export BOOTHBOT_REPO_PATH="/home/pat/a_ws/boothbot_ws/boothbot"
export ROS_MASTER_URI
export ROS_HOSTNAME
export ROS_IP

# providing bash functions
export _FILE_PATH=$(dirname $(readlink -f "$0"))

function dc_build () {
    docker-compose -f $_FILE_PATH/docker-compose.build.yml up
}

function dc () {
    docker-compose -f $_FILE_PATH/docker-compose.yml $@
}

echo "==================================USAGE=================================="
echo -e "The local env setup is \e[0;31m$_SETUP_DONE\e[0m"
echo    "Run 'dc_build' to do 'catkin build -cs' inside docker:"
echo    "Use 'dc' + command to run as docker-compose with specific file"
echo    "Such as 'dc up' to start the simulation"
echo    "    ROS_MASTER_URI: \e[0;31m$ROS_MASTER_URI\e[0m"
echo    "      ROS_HOSTNAME: \e[0;31m$ROS_HOSTNAME\e[0m"
echo    "            ROS_IP: \e[0;31m$ROS_IP\e[0m"
echo    "    BOOTHBOT_IMAGE: \e[0;31m$BOOTHBOT_IMAGE\e[0m"
echo    "DEVICE_CONFIG_PATH: \e[0;31m$DEVICE_CONFIG_PATH\e[0m"

echo "==================================USAGE=================================="