ARG UBUNTU_VERSION=20.04

FROM ubuntu:${UBUNTU_VERSION}
ENV DEBIAN_FRONTEND=noninteractive
ARG ROS_DISTRO=noetic

SHELL ["/bin/bash", "-c"]

# Python3 needs BeautifulSoup 4, rosdep tries for Py2 python-beautifulsoup Debian package
RUN apt -y update && \
    apt -y install apt-transport-https curl emacs firefox locales lsb-core python3 python3-bs4 python-bs4-doc python3-pip qt5-default qt5-qmake qtbase5-dev-tools libqt5gui5 libqt5svg5 qt5-gtk-platformtheme qt5-image-formats-plugins vim xfce4 xfce4-terminal zenity  &&\
    rm -rf /var/lib/apt/lists/*

# do we need Mesa?
RUN apt -y update && \
    apt -y install libxtst6 libegl1-mesa libgl1-mesa-dev libgl1-mesa-glx libglew-dev libosmesa6-dev &&\
    rm -rf /var/lib/apt/lists/*

RUN locale-gen en_US.UTF-8 && update-locale LANG=en_US.UTF-8

# this is the most frequently accepted solution to a problem with the Qt5 shared libs not being found
RUN strip --remove-section=.note.ABI-tag /usr/lib/x86_64-linux-gnu/libQt5Core.so

# ROS stuff starts here

RUN echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc |  apt-key add -

RUN apt -y update  && \
    apt -y install ros-${ROS_DISTRO}-desktop-full &&\
    rm -rf /var/lib/apt/lists/*

RUN apt -y update  && \
    apt -y install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool python3-catkin-pkg python3-catkin-tools python3-osrf-pycommon python3-vcstool build-essential ros-noetic-moveit ros-noetic-universal-robots ros-noetic-ros-control ros-noetic-ros-controllers && \
    rm -rf /var/lib/apt/lists/*

RUN rosdep init

# add MS VS Code
RUN cd /tmp && \
    curl -s https://packages.microsoft.com/keys/microsoft.asc |  gpg --dearmor > packages.microsoft.gpg && \
    install -D -o root -g root -m 644 packages.microsoft.gpg /etc/apt/keyrings/packages.microsoft.gpg && \
    echo "deb [arch=amd64,arm64,armhf signed-by=/etc/apt/keyrings/packages.microsoft.gpg] https://packages.microsoft.com/repos/code stable main" > /etc/apt/sources.list.d/vscode.list

RUN apt -y update && \
    apt -y install code && \
    rm -rf /var/lib/apt/lists/*

# install VirtualGL, probably not helpful from inside container but was in legacy container
RUN apt -y update && \
    apt -y install bumblebee primus-libs && \
    rm -rf /var/lib/apt/lists/*

# expect a command to run when this container invoked on the command line
    
