FROM ros:noetic-ros-base-focal

ENV USERNAME=augbooth
ENV HOME=/home/${USERNAME}

# Use bash to replace sh
RUN rm /bin/sh && ln -s /bin/bash /bin/sh

# Add new user
RUN useradd -m ${USERNAME} && \
    usermod -aG sudo ${USERNAME} && \
    usermod -aG dialout ${USERNAME} && \
    usermod -aG video ${USERNAME} && \
    echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

# Create catkin_ws and other required directory
RUN mkdir -p ${HOME}/catkin_ws/src && \
    mkdir -p ${HOME}/.ros && \
    chown -R ${USERNAME}:${USERNAME} ${HOME}/catkin_ws && \
    chown -R ${USERNAME}:${USERNAME} ${HOME}/.ros

# Install system-wide deps
RUN apt-get update && apt-get install -y \
    ros-noetic-rospy-message-converter \
    python3-pip \
    curl

# Install python deps
COPY ./requirements.txt /tmp
RUN sudo -H pip install -r /tmp/requirements.txt

# Add build scripts
COPY scripts ${HOME}/docker/scripts
RUN chown -R ${USERNAME}:${USERNAME} ${HOME}/docker/scripts && \
    chmod +x ${HOME}/docker/scripts/*.bash

# Swithc user and working dir
USER ${USERNAME}
WORKDIR ${HOME}/catkin_ws/

# Install boothbot fleet
RUN source /opt/ros/noetic/setup.bash && catkin init && catkin build

# Add entrypoint
COPY ros_entrypoint.sh /
RUN sudo chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]

CMD ["bash"]