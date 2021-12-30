# Setups for your own pc
export _SETUP_DONE="Not done!"

# CAUSION: make sure the folders on blow are user accessable
export DEVICE_CONFIG_PATH="/home/pat/catkin_ws/local"
export ROS_HOME="$DEVICE_CONFIG_PATH/roslogs"
export BOOTHBOT_PYTHON_SITE_PACKAGES="$DEVICE_CONFIG_PATH/site-packages"
export BOOTHBOT_BACKOFFICE_LOG_PATH="$DEVICE_CONFIG_PATH/backoffice"
export CATKIN_WS_PATH="/home/pat/catkin_ws/docker/catkin_ws"
export BOOTHBOT_REPO_PATH="/home/pat/a_ws/boothbot_ws/boothbot"
export BOOTHBOT_CONFIG_REPO_PATH="/home/pat/a_ws/boothbot_ws/boothbot-config"
export AUGUSTBOT_TOOLS_REPO_PATH="/home/pat/a_ws/augustbot_ws/augustbot-tools"

export BOOTHBOT_IMAGE="augustrobotics/lionel-x86:v3.1"
export ROS_MASTER_URI
export ROS_HOSTNAME
export ROS_IP

# will warning if above dir are not existed
if [ ! -d "$DEVICE_CONFIG_PATH" ]; then
  echo "WARNING: the dir $DEVICE_CONFIG_PATH is not existed, please create the dir first!!"
fi
if [ ! -d "$BOOTHBOT_PYTHON_SITE_PACKAGES" ]; then
  echo "WARNING: the dir $BOOTHBOT_PYTHON_SITE_PACKAGES is not existed, please create the dir first!!"
fi
if [ ! -d "$BOOTHBOT_BACKOFFICE_LOG_PATH" ]; then
  echo "WARNING: the dir $BOOTHBOT_BACKOFFICE_LOG_PATH is not existed, please create the dir first!!"
fi
if [ ! -d "$ROS_HOME" ]; then
  echo "WARNING: the dir $ROS_HOME is not existed, please create the dir first!!"
fi
if [ ! -d "$CATKIN_WS_PATH" ]; then
  echo "WARNING: the dir $CATKIN_WS_PATH is not existed, please create the dir first!!"
fi
if [ ! -d "$BOOTHBOT_REPO_PATH" ]; then
  echo "WARNING: the dir $BOOTHBOT_REPO_PATH is not existed, please create the dir first!!"
fi
if [ ! -d "$BOOTHBOT_CONFIG_REPO_PATH" ]; then
  echo "WARNING: the dir $BOOTHBOT_CONFIG_REPO_PATH is not existed, please create the dir first!!"
fi
if [ ! -d "$AUGUSTBOT_TOOLS_REPO_PATH" ]; then
  echo "WARNING: the dir $AUGUSTBOT_TOOLS_REPO_PATH is not existed, please create the dir first!!"
fi

# providing bash functions
export _FILE_PATH=$(dirname $(readlink -f "$0"))

function dc_build () {
    docker-compose -f $_FILE_PATH/docker-compose.build.yml up
}

function dc () {
    docker-compose -f $_FILE_PATH/docker-compose.yml $@
}

function dc_debug () {
    docker exec -it roscore bash -c "source /home/augbooth/docker/scripts/debug.bash && bash"
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
