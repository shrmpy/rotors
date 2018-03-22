#!/usr/bin/env bash
set -e

if [ "$EUID" -ne 0 ]; then
    echo "This script uses functionality which requires root privileges"
    exit 1
fi

# Assign VNC password via env var
VNC_PASSWORD=secret
##NAME=gopher
##HOME=/home/$NAME

# base image 
acbuild --debug begin docker://osrf/ros:lunar-desktop-full-xenial

# In the event of the script exiting, end the build
trap "{ export EXT=$?; acbuild --debug end && exit $EXT; }" EXIT

# Install dependencies
acbuild run -- apt-get update
acbuild run -- apt-get install -y \
            libgl1-mesa-glx libgl1-mesa-dri mesa-utils \
            dbus-x11 x11-utils x11vnc xvfb supervisor \
            dwm suckless-tools dmenu stterm \
            ros-lunar-joy ros-lunar-octomap-ros ros-lunar-mavlink ros-lunar-catkin protobuf-compiler libgoogle-glog-dev ros-lunar-control-toolbox \
            python-pip python-setuptools
acbuild run -- pip2 install future
acbuild run -- x11vnc -storepasswd $VNC_PASSWORD /etc/vncsecret
acbuild run -- chmod 444 /etc/vncsecret
acbuild run -- apt-get autoclean
acbuild run -- apt-get autoremove
acbuild run -- rm -rf /var/lib/apt/lists/* 

# Make the container's entrypoint the super
acbuild run -- mkdir -p /etc/supervisor/conf.d
acbuild copy supervisord.conf /etc/supervisor/conf.d/supervisord.conf
acbuild set-exec -- /usr/bin/supervisord -c /etc/supervisor/conf.d/supervisord.conf
acbuild port add vnc tcp 5900

# Choose the user to run-as inside the container
##acbuild run -- adduser --system --home $HOME --shell /bin/bash --group --disabled-password $NAME
##acbuild run -- usermod -a -G www-data,sudo $NAME
##acbuild set-user $NAME
##acbuild set-working-directory $HOME

# Build a workspace and include RotorS 
acbuild --debug run -- mkdir -p ~/catkin_ws/src
acbuild --debug run -- /bin/sh -c "cd ~/catkin_ws/src; \
                                  . /opt/ros/lunar/setup.sh; \
                                  rosdep update; \
                                  wstool init; \
                                  curl -LO https://raw.githubusercontent.com/ethz-asl/rotors_simulator/master/rotors_hil.rosinstall; \
                                  wstool merge rotors_hil.rosinstall; \
                                  wstool update; \
                                  cd ~/catkin_ws; \
                                  catkin_make_isolated; \
                                  . ~/catkin_ws/devel/setup.sh;"

# Write the result
acbuild --debug set-name shrmpy/rotors
acbuild --debug label add version 0.0.1

acbuild --debug write --overwrite rotors-0.0.1-linux-amd64.aci
