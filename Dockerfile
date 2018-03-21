FROM osrf/ros:kinetic-desktop-full-xenial

ARG VNC_PASSWORD=secret
ENV VNC_PASSWORD=${VNC_PASSWORD} \
    DEBIAN_FRONTEND=noninteractive

RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'; \
    curl -Ls http://packages.ros.org/ros.key --output - | apt-key add -
RUN apt-get update; apt-get install -y \
            libgl1-mesa-glx libgl1-mesa-dri mesa-utils \
            dbus-x11 x11-utils x11vnc xvfb supervisor \
            dwm suckless-tools dmenu stterm \
            ros-kinetic-joy ros-kinetic-octomap-ros ros-kinetic-mavlink python-catkin-tools protobuf-compiler libgoogle-glog-dev ros-kinetic-control-toolbox; \
    rosdep init; rosdep update; \
    adduser --system --home /home/gopher --shell /bin/bash --group --disabled-password gopher; \
    usermod -a -G www-data gopher; \
    mkdir -p /etc/supervisor/conf.d; \
    x11vnc -storepasswd $VNC_PASSWORD /etc/vncsecret; \
    chmod 444 /etc/vncsecret; \
    apt-get autoclean; \
    apt-get autoremove; \
    rm -rf /var/lib/apt/lists/*; 

COPY supervisord.conf /etc/supervisor/conf.d
EXPOSE 5900

USER gopher
WORKDIR /home/gopher
RUN echo "source /opt/ros/kinetic/setup.bash" >> /home/gopher/.bashrc; \
    echo "source ~/catkin_ws/devel/setup.bash" >> /home/gopher/.bashrc; \
    mkdir -p ~/catkin_ws/src; \
    cd ~/catkin_ws/src; \
    catkin_init_workspace; \
    wstool init; \
    curl -LO https://raw.githubusercontent.com/ethz-asl/rotors_simulator/master/rotors_hil.rosinstall; \
    wstool merge rotors_hil.rosinstall; \
    wstool update; \
    cd ~/catkin_ws; 
####    catkin build;


CMD ["/usr/bin/supervisord","-c","/etc/supervisor/conf.d/supervisord.conf"]

