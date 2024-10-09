#!/bin/bash

# Update and upgrade the system
sudo apt-get update && sudo apt-get dist-upgrade -y && sudo apt-get install -y \
    curl wget git cmake libgl1-mesa-glx mesa-utils python3 python3-pip \
    cmake build-essential python-is-python3 libgz-plugin-dev \
    libboost-program-options-dev libusb-1.0-0-dev \
    ros-humble-ament-cmake-python python3-transforms3d && \
    sudo rm -rf /var/lib/apt/lists/*

# Install RVIZ2
sudo apt-get install -y ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-rviz-* \
    ros-humble-cv-bridge \
    ros-humble-camera-calibration-parsers && \
    sudo rm -rf /var/lib/apt/lists/*

# Install Python dependencies
pip3 install transformations rowan transforms3d

# Install YoloV8
pip3 install opencv-python torch torchvision torchaudio ultralytics

# Install Gazebo Fortress
echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update && sudo apt-get install ignition-fortress -y
source /opt/ros/${ROS_DISTRO}/setup.bash

# Install GZ dependencies
curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update && sudo apt-get install gz-garden -y

# Create ROS workspace
source /opt/ros/humble/setup.bash
mkdir -p $HOME/ros_ws/src
cd $HOME/ros_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install

source /opt/ros/humble/setup.bash
echo "source $HOME/ros_ws/install/setup.bash" >> ~/.bashrc

# Clone and install CrazySim
cd $HOME/ros_ws/src
git clone https://github.com/gtfactslab/CrazySim.git --recursive
cd CrazySim/crazyflie-lib-python
pip3 install -e .

# Clone and install crazyflie-clients-python
cd $HOME/ros_ws/src
git clone https://github.com/llanesc/crazyflie-clients-python.git
cd crazyflie-clients-python
git checkout sitl-release
pip3 install -e .

# Install Jinja2
pip3 install Jinja2

# Build Crazyflie firmware
cd $HOME/ros_ws/src/CrazySim/crazyflie-firmware
mkdir -p sitl_make/build
cd sitl_make/build
cmake ..
make all

# Clone tf_transformations
cd $HOME/ros_ws/src && git clone https://github.com/DLu/tf_transformations.git
cd $HOME/ros_ws && source /opt/ros/humble/setup.bash && \
     rosdep update && \
     rosdep install --from-paths src --ignore-src -r -y && \
     colcon build --symlink-install

