#!/bin/bash

# ROS Configs
source "{{ catkin_workspace }}/devel/setup.bash"
export ROS_MASTER_URI="{{ ros_master_uri }}"
export ROS_IP="{{ ros_ip }}"

# Device Config path
export DEVICE_CONFIG_PATH="{{ local_config_path }}"

# Show the container name and make some indication
if [ -n "$CONTAINER_NAME" ]; then
    # in container dns in private network is not working
    PS1="\e[0;33m(docker)\u@$CONTAINER_NAME \w > \e[0m";
    return
else
    # in host dns in private network is working
    PS1='${debian_chroot:+($debian_chroot)}\[\033[01;32m\]\u@\h\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\$ ';
fi

# Aliases
alias mt="multitail -cS lionel_config"
alias build="{{ scripts_path }}/build.bash -c"
alias host_mode="{{ scripts_path }}/running_mode.bash -h"
alias docker_mode="{{ scripts_path }}/running_mode.bash -d"
alias manual_mode="{{ scripts_path }}/running_mode.bash -m; docker exec -it lionel_debug bash"
alias debug_shell="docker exec -it lionel_debug bash"
alias backoffce="docker-compose -f {{ config_file_path }}/docker/docker-compose.yml up -d backoffice"
alias bringup_hardware="docker-compose -f {{ config_file_path }}/docker/docker-compose.yml up -d bringup_hardware"
alias boothbot_main="docker-compose -f {{ config_file_path }}/docker/docker-compose.yaml up boothbot_main"

# Show all docker images and its version
echo "===================================================================================================="
echo -e "- \e[0;31mDocker Images:\e[0m"
if [ -f "/usr/bin/docker" ]; then
    docker image list;
    echo
fi

# Collect Device Information and Show
function repo_branch_name () {
    local repo_path=$1
    local branch_name
    if [ -d "$repo_path" ]; then
        branch_name=$(cd $repo_path && git rev-parse --abbrev-ref HEAD)
    else
        branch_name="Not Found"
    fi
    echo $branch_name
}

function running_mode () {
    local running_mode
    if [ -f "/lib/systemd/system/boothbot_main.service" ]; then
        running_mode="Host"
    else
        if [ "$(docker ps -q -f name=boothbot_main)" ]; then
            running_mode="Docker"
        else
            running_mode="Manual"
        fi
    fi
    echo $running_mode
}

# Device Name
SERIAL="{{ inventory_hostname }}"

# boothbot repo branch if the repo exist
REPO_PATH={{ boothbot_path }}
BOOTHBOT_BRANCH=$(repo_branch_name $REPO_PATH)

# augustbot-base repo branch if the repo exist
REPO_PATH={{ augustbot_base_path }}
AUGUSTBOT_BASE_BRANCH=$(repo_branch_name $REPO_PATH)

# boothbot-config repo branch if the repo exist
REPO_PATH={{ boothbot_config_path }}
BOOTHBOT_CONFIG_BRANCH=$(repo_branch_name $REPO_PATH)

# augustbot_tools repo branch if the repo exist
REPO_PATH={{ augustbot_tools_path }}
AUGUSTBOT_TOOLS_BRANCH=$(repo_branch_name $REPO_PATH)

# Running Mode
RUNNING_MODE=$(running_mode)

# free disk space
FREE_DISK_SPACE=$(df -h | grep  '/dev/mmcblk0p1' | awk '{print $4}')

echo "...................................................................................................."
echo -e "- \e[0;31mDevice Information:\e[0m"
echo -e "               SN : \e[0;31m$SERIAL\e[0m"
echo -e "         boothbot : \e[0;31m$BOOTHBOT_BRANCH\e[0m"
echo -e "   augustbot-base : \e[0;31m$AUGUSTBOT_BASE_BRANCH\e[0m"
echo -e "  boothbot-config : \e[0;31m$BOOTHBOT_CONFIG_BRANCH\e[0m"
echo -e "  augustbot-tools : \e[0;31m$AUGUSTBOT_TOOLS_BRANCH\e[0m"
echo    "    ROS_MASTER_URI: $ROS_MASTER_URI"
echo    "            ROS_IP: $ROS_IP"
echo    "      Running Mode: $RUNNING_MODE"
echo    "   Free Disk Space: $FREE_DISK_SPACE"

echo "...................................................................................................."
echo -e "- \e[0;31mSome Useful Commands:\e[0m"
echo "           mt: View ros logs with color. Example: 'mt -n 10000 ~/.ros/log/latest/<log file name>'."
echo "        build: Clean and build the lionel code."
echo "    host_mode: Switch to the 'host' mode(robot upstart)."
echo "  docker_mode: Switch to the 'docker' mode."
echo "  manual_mode: Switch to the 'manual' mode and connect to the docker debug shell."
echo "  debug_shell: Open a new debug shell while in 'manual' mode."
echo "===================================================================================================="
