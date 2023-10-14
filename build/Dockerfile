ARG UBUNTU_VERSION=22.04
# Jammy Jellyfish current LTS in Septembe 2023

FROM ubuntu:${UBUNTU_VERSION}
ENV DEBIAN_FRONTEND=noninteractive

# SHELL ["/bin/bash", "-c"]

RUN apt update && \
    apt -y upgrade && \
    apt -y install emacs firefox locales lsb-core python3 python3-pip vim xfce4 xfce4-terminal zenity  &&\
    rm -rf /var/lib/apt/lists/*
#  qt5-default qt5-qmake qtbase5-dev-tools libqt5gui5 libqt5svg5 qt5-gtk-platformtheme qt5-image-formats-plugins

RUN locale-gen en_US.UTF-8 && \
    update-locale LANG=en_US.UTF-8

# RUN apt -y install qt5-default qt5-qmake qtbase5-dev-tools libqt5gui5 libqt5svg5 qt5-gtk-platformtheme qt5-image-formats-plugins
# this is the most frequently accepted solution to a problem with the Qt5 shared libs not being found
# RUN strip --remove-section=.note.ABI-tag /usr/lib/x86_64-linux-gnu/libQt5Core.so

# add Google Chrome
RUN cd /tmp && \
    curl -s https://dl.google.com/linux/linux_signing_key.pub | apt-key add - && \
    echo "deb http://dl.google.com/linux/chrome/deb/ stable main" > /etc/apt/sources.list.d/google.list

RUN apt update && \
    apt -y install google-chrome-stable

# add MS VS Code
RUN cd /tmp && \
    curl -s https://packages.microsoft.com/keys/microsoft.asc |  gpg --dearmor > packages.microsoft.gpg && \
    install -D -o root -g root -m 644 packages.microsoft.gpg /etc/apt/keyrings/packages.microsoft.gpg && \
    echo "deb [arch=amd64,arm64,armhf signed-by=/etc/apt/keyrings/packages.microsoft.gpg] https://packages.microsoft.com/repos/code stable main" > /etc/apt/sources.list.d/vscode.list

RUN apt update && \
    apt -y install code && \
    rm -rf /var/lib/apt/lists/*

# expect a command to run when this container invoked on the command line
    
