#!/bin/bash

# initialize environment for ES159
# - manually replace the usual /n/academic_homes/<class_dir>/SHARE/Programs/class_setup.sh
#   with the contents of this file

PATH=/bin:/usr/bin

export SEMESTER=FALL2023
ROS_DISTRO=noetic
ROS_ENV_SOURCE=/opt/ros/${ROS_DISTRO}/setup.bash

export SHARED_DATA=/n/holyscratch01/Academic-cluster/${SEMESTER}/$(id -g -n)/SHARED
# Add symlink to a shared data folder containing datasets
if [ -e "$SHARED_DATA" ]; then
    if [ ! -e "$HOME/shared_data" ]; then
        ln -s $SHARED_DATA $HOME/shared_data
    fi
else
    echo "Could not find your class shared_data folder at ${SHARED_DATA} and cannot proceed." 1>&2
    exit 1
fi

export SCRATCH_PATH=/n/holyscratch01/Academic-cluster/${SEMESTER}/$(id -g -n)/SCRATCH
export MYSCRATCH=${SCRATCH_PATH}/$USER
if [ -e ${SCRATCH_PATH} -a -w ${SCRATCH_PATH} -a -d ${SCRATCH_PATH} ]; then
    # create the folder if it does not exists
    if [ ! -e "$MYSCRATCH" ]; then
        mkdir -m 700 $MYSCRATCH
    fi
else
    echo "Could not create your personal scratch folder and cannot proceed." 1>&2
    exit 1
fi

# Add symlink to a scratch data folder
if [ -e ${MYSCRATCH} ] && [ ! -e ${HOME}/scratch_folder ]; then
    ln -s ${MYSCRATCH} ${HOME}/scratch_folder
fi

# update ~/.bashrc so all new shells are ROS-ready with class-local packages
if [ -e ${ROS_ENV_SOURCE} -a -r ${ROS_ENV_SOURCE} ]; then

    source ${ROS_ENV_SOURCE}

    if [ ! -s "$(grep -F /opt/ros/${ROS_DISTRO}/setup.bash ~/.bashrc 2> /dev/null | grep source)" ]; then
        echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
    fi

    if [ -s "$(which catkin 2> /dev/null)" ]; then
        grep -F `catkin locate --shell-verbs` ~/.bashrc ||
        echo "source $(catkin locate --shell-verbs)" >> ~/.bashrc
    else
        echo "Could not locate the catkin command, so setup is incomplete. Please report this."
	exit 1
    fi

    ## leave the defaults in place, Robin W. 2023-09-07
    # grep -F "ROS_IP" ~/.bashrc ||
    # echo "export ROS_IP=127.0.0.1" >> ~/.bashrc

    # grep -F "ROS_MASTER_URI" ~/.bashrc ||
    # echo "export ROS_MASTER_URI=http://\$ROS_IP:11311" >> ~/.bashrc

    # with per-class Catkin Workspace in shared_data
    if [ -e ~/shared_data/catkin_ws/devel -a -r ~/shared_data/catkin_ws/devel/setup.bash ]; then
        echo "source ~/shared_data/catkin_ws/devel/setup.bash" >> ~/.bashrc
        source ~/shared_data/catkin_ws/devel/setup.bash
    else
        echo "The Catkin Workspace for the class could not be found, and has not been set up." 1>&2
    fi

    if [ -s "$(which rosdep 2> /dev/null)" ]; then
        # users cannot 'sudo rosdep init' so assume that's done and just update local cache
        rosdep update 2>&1 > /dev/null
    else
        echo "Could not find rosdep command! Installation is incomplete. Please report this." 1>&2
    fi

    echo ""
    echo "Completed setting up the class, unless there was an error message."
    echo "Please contact a teaching assistant about any error messages or if you have further questions."
    echo "If there were no problems reported from the last command,"
    echo "Please run 'source ~/.bashrc' in this window."
    echo ""

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

