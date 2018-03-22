FROM osrf/ros:lunar-desktop-full-xenial

ARG VNC_PASSWORD=secret
ENV VNC_PASSWORD=${VNC_PASSWORD} \
    LOGIN=rotors \
    HOME=/home/rotors \
    DEBIAN_FRONTEND=noninteractive

RUN apt-get update; apt-get install -y \
            libgl1-mesa-glx libgl1-mesa-dri mesa-utils \
            dbus-x11 x11-utils x11vnc xvfb supervisor \
            dwm suckless-tools stterm \
            ros-lunar-joy ros-lunar-octomap-ros ros-lunar-mavlink protobuf-compiler libgoogle-glog-dev ros-lunar-control-toolbox \
            python-pip python-setuptools; \
    pip2 install future; \
    adduser --system --home ${HOME} --shell /bin/bash --group --disabled-password $LOGIN; \
    usermod -a -G www-data $LOGIN; \
    mkdir -p /etc/supervisor/conf.d; \
    x11vnc -storepasswd $VNC_PASSWORD /etc/vncsecret; \
    chmod 444 /etc/vncsecret; \
    apt-get autoclean; \
    apt-get autoremove; \
    rm -rf /var/lib/apt/lists/*; 

COPY supervisord.conf /etc/supervisor/conf.d
EXPOSE 5900
CMD ["/usr/bin/supervisord","-c","/etc/supervisor/conf.d/supervisord.conf"]

USER $LOGIN
WORKDIR ${HOME}

# Build a workspace and include RotorS 
RUN mkdir -p ${HOME}/catkin_ws/src; \
    cd ${HOME}/catkin_ws/src; \
    . /opt/ros/lunar/setup.sh; \
    rosdep update; \
    wstool init; \
    curl -sLO https://raw.githubusercontent.com/ethz-asl/rotors_simulator/master/rotors_hil.rosinstall; \
    wstool merge rotors_hil.rosinstall; \
    wstool update; \
    cd ${HOME}/catkin_ws; \
    catkin_make_isolated; \
    . ${HOME}/catkin_ws/devel/setup.sh; 
##    echo ". ${HOME}/catkin_ws/devel/setup.sh" >> ${HOME}/.bashrc; 

####$ roslaunch rotors_gazebo mav_hovering_example.launch mav_name:=firefly world_name:=basic
