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

