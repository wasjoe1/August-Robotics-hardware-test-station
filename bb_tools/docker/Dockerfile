## For x86 Processor
FROM osrf/ros:melodic-desktop-full

#TODO: This should be follow the method used in cicd, use the most up to date image
## For ARM Processor
# FROM nvcr.io/nvidian/nvidia-l4t-base:r32.6

ENV USERNAME=augbooth
ENV HOME /home/${USERNAME}

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
    chown -R ${USERNAME}:${USERNAME} ${HOME}/.ros && \
    mkdir -p /var/log/boothbot-backoffice && \
    touch /var/log/boothbot-backoffice/django_info.log && \
    chown -R ${USERNAME}:${USERNAME} /var/log/boothbot-backoffice/django_info.log

# Install system-related dependences
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        software-properties-common \
        build-essential \
        gfortran \
        cmake \
        git \
        curl \
        file \
        tar \
        python-dev \
        python-pip \
        python3-dev \
        python3-pip \
        python3-numpy \
        python3-distutils \
        python3-setuptools \
        libffi-dev \
        python-catkin-tools \
        libgoogle-glog-dev \
    && rm -rf /var/lib/apt/lists/*

# Add apt-key for ros packages
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

# Install ros-related dependences
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        ros-melodic-image-transport \
        ros-melodic-vision-msgs \
        ros-melodic-yocs-cmd-vel-mux \
        ros-melodic-robot-localization \
        ros-melodic-spatio-temporal-voxel-layer \
        ros-melodic-range-sensor-layer \
        ros-melodic-serial \
        ros-melodic-robot-upstart \
        ros-melodic-map-server \
        ros-melodic-move-base \
        ros-melodic-global-planner \
        ros-melodic-teb-local-planner \
        ros-melodic-rospy-message-converter \
        ros-melodic-yocs-velocity-smoother \
        ros-melodic-bfl \
        python-rosdep \
        python-rosinstall \
        python-rosinstall-generator \
        python-wstool \
        iproute2 \
        libncurses5-dev \
    && rm -rf /var/lib/apt/lists/*

# Install python2 packages
COPY ./packages/requirements_py2.txt ${HOME}
RUN pip2 install --no-cache-dir --upgrade pip setuptools wheel && \
    pip2 install --no-cache-dir -r ${HOME}/requirements_py2.txt && \
    rm -f ${HOME}/requirements_py2.txt

RUN grep -qxF "source /opt/ros/melodic/setup.bash" ~/.bashrc || echo 'source /opt/ros/melodic/setup.bash' >> ~/.bashrc;

# Alternate way to install backoffice
RUN mkdir -p ${HOME}/.local/lib/python2.7/site-packages && \
    echo "/home/augbooth/catkin_ws/src/boothbot/backoffice" > ${HOME}/.local/lib/python2.7/site-packages/BoothBot-Backoffice.egg-link && \
    echo "/home/augbooth/catkin_ws/src/boothbot/backoffice" > ${HOME}/.local/lib/python2.7/site-packages/easy-install.pth && \
    chown -R ${USERNAME}:${USERNAME} ${HOME}/.local

# Entrypoint for ros
COPY ./scripts/ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod 755 /ros_entrypoint.sh

COPY ./scripts/build.bash /build.bash
RUN chmod 755 /build.bash

USER ${USERNAME}
WORKDIR ${HOME}

CMD ["/bin/bash", "source", " ~/.bashrc"]
ENTRYPOINT ["/bin/bash", "/ros_entrypoint.sh"]