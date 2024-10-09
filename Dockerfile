FROM osrf/ros:humble-desktop-full

ARG DEBIAN_FRONTEND=noninteractive
WORKDIR /home/ubuntu
# Install dependencies
SHELL ["/bin/bash", "-c"]
RUN apt-get update && apt-get dist-upgrade -y && apt-get install -y \
    curl wget git cmake libgl1-mesa-glx mesa-utils python3 python3-pip \
    cmake build-essential python-is-python3 libgz-plugin-dev \
    libboost-program-options-dev libusb-1.0-0-dev \
    ros-humble-ament-cmake-python python3-transforms3d && \
    rm -rf /var/lib/apt/lists/*
    
# Install RVIZ2
RUN apt-get install -y ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-rviz-* \
    ros-humble-cv-bridge \
    ros-humble-camera-calibration-parsers \
    && rm -rf /var/lib/apt/lists/*

# Install PYTHON dependencies
RUN pip3 install transformations rowan transforms3d
# Install YoloV8
RUN pip3 install opencv-python torch torchvision torchaudio ultralytics

# Install gazebo fortress
RUN echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list
RUN wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add -
RUN apt-get update && apt-get install ignition-fortress -y
RUN source /opt/ros/${ROS_DISTRO}/setup.bash

# GZ dependencies
RUN curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
RUN apt-get update && apt-get install gz-garden -y

# Create workspace
RUN source /opt/ros/humble/setup.bash && \ 
    mkdir -p /home/ubuntu/ros_ws/src && \
    cd /home/ubuntu/ros_ws && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    colcon build --symlink-install

RUN source /opt/ros/humble/setup.bash && \
    # echo source /usr/share/gazebo/setup.bash >> ~/.bashrc && \
    echo source /home/ubuntu/ros_ws/install/setup.bash >> ~/.bashrc

# CrazySim
RUN cd /home/ubuntu/ros_ws/src && \
    git clone https://github.com/gtfactslab/CrazySim.git --recursive && \
    cd ./CrazySim/crazyflie-lib-python && \
    pip3 install -e .

RUN cd /home/ubuntu/ros_ws/src && \
    git clone https://github.com/llanesc/crazyflie-clients-python.git && \
    cd ./crazyflie-clients-python && \
    git checkout sitl-release && \
    pip3 install -e .

RUN pip3 install Jinja2

RUN cd /home/ubuntu/ros_ws/src/CrazySim/crazyflie-firmware && \
    mkdir -p sitl_make/build && cd $_ && \
    cmake .. && \
    make all

RUN cd /home/ubuntu/ros_ws/src && \
    git clone https://github.com/DLu/tf_transformations.git


# RUN source /opt/ros/humble/setup.bash && \
#      cd /home/ubuntu/ros_ws && \
#      rosdep update && \
#      rosdep install --from-paths src --ignore-src -r -y && \
#      colcon build --symlink-install

RUN source ~/.bashrc