#!/bin/bash

# initialize environment for ES159
# - manually replace the usual /n/academic_homes/<class_dir>/SHARE/Programs/class_setup.sh
#   with the contents of this file

PATH=/bin:/usr/bin

SEMESTER=FALL2023
ROS_DISTRO=noetic
ROS_ENV_SOURCE=/opt/ros/${ROS_DISTRO}/setup.bash

SHARED_DATA=/n/holyscratch01/Academic-cluster/${SEMESTER}/$(id -g -n)/SHARED
# Add symlink to a shared data folder containing datasets
if [ -e "$SHARED_DATA" ]; then
    if  [ ! -e "$HOME/shared_data" ]; then
        ln -s $SHARED_DATA $HOME/shared_data
    fi
else
    echo "Could not find your class shared_data folder at ${SHARED_DATA} and cannot proceed with robotics environment setup." 1>&2
    exit 1
fi

SCRATCH_PATH=/n/holyscratch01/Academic-cluster/${SEMESTER}/$(id -g -n)/SCRATCH
MYSCRATCH=${SCRATCH_PATH}/$USER

if [ -e ${SCRATCH_PATH} -a -w ${SCRATCH_PATH} -a -d ${SCRATCH_PATH} ]; then
    # create the folder if it does not exists
    if [ ! -e "$MYSCRATCH" ]; then
        mkdir -m 700 $MYSCRATCH
    fi
else
    echo "Could not create your personal scratch folder and cannot proceed with robotics environment setup." 1>&2
    exit 1
fi

# Add symlink to a scratch data folder
if [ -e ${MYSCRATCH} ] && [ ! -e ${HOME}/scratch_folder ]; then
    ln -s ${MYSCRATCH} ${HOME}/scratch_folder
fi

# update ~/.bashrc so all new shells are ROS-ready with class-local packages
if [ -e ${ROS_ENV_SOURCE} -a -r ${ROS_ENV_SOURCE} ]; then

    grep -F "${ROS_ENV_SOURCE}" ~/.bashrc ||
    echo "source ${ROS_ENV_SOURCE}" >> ~/.bashrc

    source ${ROS_ENV_SOURCE}

#    if [ -s "$(which catkin 2> /dev/null)" ]; then
#        grep -F `catkin locate --shell-verbs` ~/.bashrc ||
#        echo "source $(catkin locate --shell-verbs)" >> ~/.bashrc
#        source $(catkin locate --shell-verbs)
#    else
#        echo "Could not locate the catkin command, so setup is incomplete." 1>&2
#	exit 1
#    fi

    ## leave the defaults in place, Robin W. 2023-09-07
    # grep -F "ROS_IP" ~/.bashrc ||
    # echo "export ROS_IP=127.0.0.1" >> ~/.bashrc

    # grep -F "ROS_MASTER_URI" ~/.bashrc ||
    # echo "export ROS_MASTER_URI=http://\$ROS_IP:11311" >> ~/.bashrc

    # with per-class Catkin Workspace in shared_data
    if [ -d ~/shared_data/catkin_ws/devel -a -r ~/shared_data/catkin_ws/devel/setup.bash ]; then
        grep -F 'source ~/shared_data/catkin_ws/devel/setup.bash' ~/.bashrc ||
        echo "source ~/shared_data/catkin_ws/devel/setup.bash" >> ~/.bashrc
        source ~/shared_data/catkin_ws/devel/setup.bash
    else
        echo "The Catkin Workspace for the class could not be found, and has not been set up." 1>&2
    fi
else
    echo "Could not find ROS ${ROS_DISTRO} setup.bash file. Is ROS insalled on this computer?" 1>&2
fi


# if we were trying to set up per student
#  mkdir -p ~/catkin_ws/src
#  cd ~/catkin_ws/
#  echo "run catkin_make to initialize the Catkin workspace"
#  catkin_make
#  source devel/setup.bash
#  echo $ROS_PACKAGE_PATH # this should show the /n/academic_homes/g124803/u...g124803/catkin_ws/src path component from the previous step
#  echo "copy in the ES 159 ROS packages"
#  cp ~/SHARE/shared_data/es159-package ~/catkin_ws/src
#  cd ~/catkin_ws
#  echo "run catkin_make to process the new ES 159 ROS packages"
#  catkin_make

