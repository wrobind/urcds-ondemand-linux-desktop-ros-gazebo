ARG UBUNTU_VERSION=20.04

FROM ubuntu:${UBUNTU_VERSION}
ENV DEBIAN_FRONTEND=noninteractive
ARG ROS_DISTRO=noetic
ARG ROBOTIC_BRANCH=melodic-devel

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
    apt -y install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool python3-catkin-pkg python3-catkin-tools python3-osrf-pycommon python3-vcstool build-essential ros-noetic-moveit ros-noetic-universal-robots &&\
    rm -rf /var/lib/apt/lists/*

RUN rosdep init && \
    rosdep update && \
    rosdep fix-permissions

# ACOD create file in /etc/profile.d for ROS environment
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /etc/bash.bashrc && \
    echo "source $(catkin locate --shell-verbs)" >> /etc/bash.bashrc

# class-shared Catkin Workspace setup in /opt/catkin_ws
#
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    mkdir -p /opt/catkin_ws/src && \
    cd /opt/catkin_ws && \
    catkin_make

# Universal Robot installation
# install Universal Robot per https://github.com/ros-industrial/universal_robot README
#  use cloned repo in 'wrobind' to allow extra packages to be added
#
RUN cd /opt/catkin_ws/src && \
    git clone -b ${ROBOTIC_BRANCH} https://github.com/wrobind/es159-univeral-robots-fall2023.git && \
    mv es159-univeral-robots-fall2023 universal_robot
#    git clone https://github.com/ros-industrial/universal_robot.git

# ENV PYTHONPATH=${PYTHONPATH}:${ROS_ROOT}/core/roslib/src

RUN cd /opt/catkin_ws && \
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    rosdep install --rosdistro ${ROS_DISTRO} --ignore-src --from-paths src && \
    catkin_make install

# should run the next command as part of above catkin_make RUN, but it messes
#  up looking for python-beautifulsoup when Python3 needs python3-bs4
#    rosdep install --rosdistro ${ROS_DISTRO} --ignore-src --from-paths src && \

#    source /opt/ros/${ROS_DISTRO}/setup.bash && \
#    rosdep update && \
#    rosdep fix-permissions && \
#    rosdep install --rosdistro ${ROS_DISTRO} --ingore-src --from-paths src && \
#    catkin_make

# add MS VS Code

RUN cd /tmp && \
    curl -s https://packages.microsoft.com/keys/microsoft.asc |  gpg --dearmor > packages.microsoft.gpg && \
    install -D -o root -g root -m 644 packages.microsoft.gpg /etc/apt/keyrings/packages.microsoft.gpg && \
    echo "deb [arch=amd64,arm64,armhf signed-by=/etc/apt/keyrings/packages.microsoft.gpg] https://packages.microsoft.com/repos/code stable main" > /etc/apt/sources.list.d/vscode.list

# follows the curl above if using original MS approach of dearmoring key and sticking it in a file
# RUN cd /tmp && \
#    install -D -o root -g root -m 644 packages.microsoft.gpg /etc/apt/keyrings/packages.microsoft.gpg && \
#    rm -f packages.microsoft.gpg
#    curl -s https://packages.microsoft.com/keys/microsoft.asc | apt-key add - && \

RUN apt -y update && \
    apt -y install code && \
    rm -rf /var/lib/apt/lists/*

#COPY ./code_1.81.1-1691620686_amd64.deb /tmp
#RUN apt -y update  && \
#    apt -y install  /tmp/code_1.81.1-1691620686_amd64.deb

# add VirtualGL from distro -- not clear if we need it, maybe for Matlab?
RUN apt -y update && \
    apt -y install bumblebee primus-libs && \
    rm -rf /var/lib/apt/lists/*


# do we need these? Robin W.
# seems like Xserver and all graphics rendering is on host
# maybe for Matlab
# COPY ./virtualgl_3.1_amd64.deb /tmp
# RUN dpkg -i /tmp/virtualgl_3.1_amd64.deb && rm -f /tmp/virtualgl_3.1_amd64.deb
##

# expect a command to run when this container invoked on the command line
    
