#!/bin/bash

set -e

# Get VERSION_CODENAME from /etc/os-release
DISTRO=$(grep '^VERSION_CODENAME=' /etc/os-release | cut -d'=' -f2)

if [ $# -ne 1 ]; then
    echo "Please set an argument to specify which ROS One installation method to use:"
    echo "0: From https://github.com/v4hn/ros-o-builder / 1: From https://ros.packages.techfak.net/"
    exit 1
fi

# Conditional branching based on distro_codename
if [ "$DISTRO" = "jammy" ] && [ "$1" = "0" ]; then
    echo "This is Ubuntu 22.04 (jammy). Install ROS-O from https://github.com/v4hn/ros-o-builder"

    sudo apt install -y python3-pip
    pip3 install catkin-tools
    echo "deb [trusted=yes] https://raw.githubusercontent.com/v4hn/ros-o-builder/$DISTRO-one-unstable/repository ./" | sudo tee /etc/apt/sources.list.d/ros-o-builder.list
    sudo apt update
    sudo apt install -y python3-rosdep2
    echo "yaml https://raw.githubusercontent.com/v4hn/ros-o-builder/$DISTRO-one-unstable/repository/local.yaml debian" | sudo tee /etc/ros/rosdep/sources.list.d/1-ros-o-builder.list
    rosdep update
    sudo apt install -y ros-one-desktop-full
    sudo apt install -y python3-wstool
    sudo pip3 install -U catkin_tools
fi

if [ "$DISTRO" = "jammy" ] && [ "$1" = "1" ]; then
    echo "This is Ubuntu 22.04 (jammy). Install ROS-O from https://ros.packages.techfak.net/"

    # Configure ROS One apt repository
    sudo apt install -y curl
    sudo curl -sSL https://ros.packages.techfak.net/gpg.key -o /etc/apt/keyrings/ros-one-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/ros-one-keyring.gpg] https://ros.packages.techfak.net $(lsb_release -cs)-testing main" | sudo tee /etc/apt/sources.list.d/ros1.list
    echo "# deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/ros-one-keyring.gpg] https://ros.packages.techfak.net $(lsb_release -cs)-testing main-dbg" | sudo tee -a /etc/apt/sources.list.d/ros1.list

    # Install and setup rosdep
    # Do not install python3-rosdep2, which is an outdated version of rosdep shipped via the Ubuntu repositories (instead of ROS)!
    sudo apt update
    sudo apt install -y python3-rosdep
    sudo rosdep init

    # Define custom rosdep package mapping
    echo "yaml https://ros.packages.techfak.net/ros-one.yaml one" | sudo tee /etc/ros/rosdep/sources.list.d/1-ros-one.list
    rosdep update

    # Install packages, e.g. ROS desktop
    sudo apt install -y ros-one-desktop

    sudo apt install -y python3-wstool
    pip3 install -U catkin_tools
fi
