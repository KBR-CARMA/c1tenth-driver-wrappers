
# Overview

New repository for housing the build system to package up all drivers for the c1tenth project, plus wrapper node to expose them to the CARAM system.

# Setting up the environment
## Clone/switch to the wrappers repo and switch to your feature branch
## code you are working on should be under c1tenth-driver-wrappers
```sh
$ cd ~/ws/c1tenth-driver-wrappers
$ git pull
$ git checkout <my-feature-branch> 
```

# Launching the CARMA container 

Launch docker c1tenth-develop using your c1tenth-driver-wrappers folder 
as a carma volume alias and bash into it

```sh
$ cd ~
$ docker run -it --rm --name dev \
--device /dev/sensors/imu \
--device /dev/sensors/vesc \
--device /dev/sensors/rplidar \
--device /dev/input/js0 \
 -v $PWD/c1tenth-driver-wrappers:/home/carma/c1tenth-driver-wrappers quitter.tech/carma-platform:c1tenth-develop bash
```
## The prompt should now look something like this
```sh
carma@a1d099b96ab7:/$
```
## Drop into a NEW terminal session to avoid a stale ROS source
```sh
$ docker exec -it dev bash
carma@7ab45d76dce2:/$
```
## Source the container's ROS2
```sh
carma$ cd ~
carma$ source /opt/carma/install_ros2/setup.bash
```
## Create a workspace inside the container
```sh 
carma$ mkdir -p ~/tmp_ws/src  
carma$ cd ~/tmp_ws/src
```
## Create a symlink to the package c1tenth-driver-wrappers
```sh 
carma$ ln -s /home/carma/c1tenth-driver-wrappers 
```
## Build the packages
```sh
carma$ cd ~/tmp_ws
carma$ sudo apt update
carma$ rosdep update
carma$ rosdep install --from-paths src --ignore-src -r -y
carma$ colcon build --packages-up-to my_driver_wrapper
```
## Source the newly built package
```sh
carma$ source install/setup.bash
```
## Launch the package
```sh
carma$ ros2 launch my_driver_wrapper my_driver_wrapper_launch.py
```
Or if you are using a params file
```sh
ros2 launch my_driver_wrapper my_driver_wrapper_launch.py --ros-args --params-file ./src/my_driver/my_driver/params/my_driver_params.yaml
```