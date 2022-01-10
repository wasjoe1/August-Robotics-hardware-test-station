
# providing bash functions
export _FILE_DIR_PATH=$(dirname $(readlink -f "$0"))
################################################################################
# For developer, modify with your local env
# Setups for your own pc
export _SETUP_DONE="Not done!"

# CAUSION: make sure the folders on blow are user accessable
export DOCKER_CONTENT_PATH="/home/pat/catkin_ws/docker"
export DEVICE_SETTING_PATH="$_FILE_DIR_PATH/config/sample"

## -=Autogenerable by `dc_gen_docker_mirror_dir`=-
export BOOTHBOT_BACKOFFICE_LOG_PATH="$DOCKER_CONTENT_PATH/backoffice_log"
export BOOTHBOT_PYTHON_SITE_PACKAGES="$DOCKER_CONTENT_PATH/site-packages"
export DOCKER_CATKIN_WS_MIRROR_PATH="$DOCKER_CONTENT_PATH/catkin_ws"
export ROS_HOME="$DOCKER_CONTENT_PATH/catkin_ws/local/roslogs"
export BOOTHBOT_MAPS="$DOCKER_CONTENT_PATH/catkin_ws/local/maps"
## -=Autogenerable by `dc_gen_docker_mirror_dir`=-

export BOOTHBOT_REPO_PATH="/home/pat/catkin_ws/src/boothbot"
export BOOTHBOT_CONFIG_REPO_PATH="/home/pat/catkin_ws/src/boothbot-config"
export AUGUSTBOT_TOOLS_REPO_PATH="/home/pat/catkin_ws/src/augustbot-tools"

export BOOTHBOT_IMAGE="augustrobotics/lionel-x86:v3.1"
export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=localhost
export ROS_IP
################################################################################

# will warning if above dir are not existed
if [ ! -d "$DOCKER_CONTENT_PATH" ]; then
  echo "WARNING: the dir $DOCKER_CONTENT_PATH is not existed, please create the dir first!!"
else
  echo "Checked that $DOCKER_CONTENT_PATH is existed, you could use 'dc_gen_docker_mirror_dir' to generate default dir under it for running docker"
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

function dc_gen_docker_mirror_dir () {
  echo "Creating $BOOTHBOT_BACKOFFICE_LOG_PATH"
  mkdir -p $BOOTHBOT_BACKOFFICE_LOG_PATH
  echo "Creating $BOOTHBOT_PYTHON_SITE_PACKAGES"
  mkdir -p $BOOTHBOT_PYTHON_SITE_PACKAGES
  echo "Creating $DOCKER_CATKIN_WS_MIRROR_PATH"
  mkdir -p $DOCKER_CATKIN_WS_MIRROR_PATH
  echo "Creating backoffice local settings folder"
  mkdir -p $DOCKER_CATKIN_WS_MIRROR_PATH/local/backoffice
  echo "Creating $ROS_HOME"
  mkdir -p $ROS_HOME
  echo "Creating $BOOTHBOT_MAPS"
  mkdir -p $BOOTHBOT_MAPS
}

function dc_build () {
    docker-compose -f $_FILE_DIR_PATH/docker-compose.build.yml up build
}

function dc () {
    docker-compose -f $_FILE_DIR_PATH/docker-compose.yml $@
}

function dc_debug () {
    docker exec -it roscore bash -c "source /home/augbooth/docker/scripts/debug.bash && bash"
}

echo "==================================USAGE=================================="
echo -e "The local env setup is \e[0;31m$_SETUP_DONE\e[0m"
echo    "Run 'dc_build' to do 'catkin build -cs' inside docker"
echo    "Use 'dc' + command to run as docker-compose with specific file"
echo    "Run 'dc_debug' to attach 'roscore' container for debugging."
echo    "Such as 'dc up' to start the simulation"
echo    "     ROS_MASTER_URI: \e[0;31m$ROS_MASTER_URI\e[0m"
echo    "       ROS_HOSTNAME: \e[0;31m$ROS_HOSTNAME\e[0m"
echo    "             ROS_IP: \e[0;31m$ROS_IP\e[0m"
echo    "     BOOTHBOT_IMAGE: \e[0;31m$BOOTHBOT_IMAGE\e[0m"
echo    "DEVICE_SETTING_PATH: \e[0;31m$DEVICE_SETTING_PATH\e[0m"
echo "==================================USAGE=================================="
