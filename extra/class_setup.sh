#!/bin/bash

export SEMESTER=FALL2023

export SHARED_DATA=/n/holyscratch01/Academic-cluster/${SEMESTER}/$(id -g -n)/SHARED
# Add symlink to a shared data folder containing datasets
if [ -e "$SHARED_DATA" ] && [ ! -e "$HOME/shared_data" ]; then
    ln -s $SHARED_DATA $HOME/shared_data
fi

export MYSCRATCH=/n/holyscratch01/Academic-cluster/${SEMESTER}/$(id -g -n)/SCRATCH/$USER
# create the folder if it does not exists
if [ ! -e "$MYSCRATCH" ]; then
    mkdir -m 700 $MYSCRATCH
fi
# Add symlink to a scratch data folder
if [ -e "$MYSCRATCH" ] && [ ! -e "$HOME/scratch_folder" ]; then
    ln -s $MYSCRATCH $HOME/scratch_folder
fi
