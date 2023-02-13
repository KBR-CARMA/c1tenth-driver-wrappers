
# Overview

New repository for housing the build system to package up all drivers for the c1tenth project, plus wrapper node to expose them to the CARAM system.


# Setting up the environment
## Clone/switch to the wrappers repo
```sh
$ git clone git@github.com:KBR-CARMA/c1tenth-driver-wrappers.git
# or
cd ~/ws/c1tenth-driver-wrappers
```
## Get latest 
```sh
$ git pull
```
## Switch to your feature branch
```sh
$ git checkout <my-feature-branch> 
```
## Go back to your workspace 
```sh
$ cd ..
```

# Launching the CARMA container 

Launch docker c1tenth-develop using your local wrappers folder 
as a carma user volume alias and bash into it

```sh
$ docker run -it --rm -v $PWD/c1tenth-driver-wrappers:/home/carma/c1tenth-driver-wrappers quitter.tech/carma-platform:c1tenth-develop bash
```
## The prompt should now look something like this
```sh
carma@a1d099b96ab7:/$
```

## Create a workspace inside the container
```sh 
carma $ mkdir -p ~/tmp_ws/src
carma $ cd ~/tmp_ws/src
```
## Create a symlink to the package c1tenth-driver-wrappers -> /home/carma/c1tenth-driver-wrappers/
```sh 
carma $ ln -s /home/carma/c1tenth-driver-wrappers 
```
## Return to the workspace
```sh
carma $ cd ..
```
## Source the containers ROS2
```sh
carma $ source /opt/carma/install_ros2/setup.bash
```
# Building the wrapper code

## Verify that your package is in the ROS2 path
```sh
carma $ colcon list
```
`my_driver_wrapper`
## Build the package
```sh
carma $ colcon build

Starting >>> my_driver_wrapper
[Processing: my_driver_wrapper]
.
.
.
Finished <<< my_driver_wrapper
```
## Source the package
```sh
carma $ source install/setup.bash
```
## Launch the package
```sh
carma $ ros2 launch my_driver_wrapper my_driver_wrapper_launch.py
```