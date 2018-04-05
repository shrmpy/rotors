Pre-configure image to do RotorS Gazebo simulation
======

This starts with the Ubuntu Xenial Lunar tag of [ROS](https://registry.hub.docker.com/osrf/ros/) base image, and follows the install steps from
 [RotorS](https://github.com/ethz-asl/rotors_simulator/) 


Quickstart
------

Specify your own VNC password for the container:

```
    clone https://github.com/shrmpy/rotors.git && cd rotors && \
    docker build --build-arg VNC_PASSWORD=MyPassGoesHere --network=host --add-host=$HOSTNAME:127.0.0.1 -t rotors .
```

Run your container (e.g., with a mounted subdir to save screenshots):

```
    mkdir Pictures && docker run --network=host --rm -ti -v $(pwd)/Pictures:/home/gopher/.gazebo/pictures rotors
```


Connect with VNC viewer, and enter the VNC password from the step above when prompted:

```
    vncviewer vnc://localhost:5900
```


Once inside the container, start Gazebo:

```
    Shift-Alt-Enter                           #(hot-key st term)
    $ bash                                    #(bash shell to dot-source)
    $ roslaunch rotors_gazebo mav_hovering_example.launch
```


