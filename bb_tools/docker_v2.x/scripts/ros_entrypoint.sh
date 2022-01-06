#!/bin/bash
set -e

# setup ros environment
sudo chown -R ${USERNAME}:${USERNAME} ${HOME};
sudo chown -R ${USERNAME}:${USERNAME} /dev;

cat >> ~/.bashrc << EOT
# Show the container name and make some indication
if [ -n "$CONTAINER_NAME" ]; then
    PS1="\e[0;33m(docker)\u@$CONTAINER_NAME \w > \e[0m";
else
    PS1='${debian_chroot:+($debian_chroot)}\[\033[01;32m\]\u@\h\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\$ ';
fi
EOT

source "/opt/ros/$ROS_DISTRO/setup.bash";
source ~/.bashrc;
exec "$@"