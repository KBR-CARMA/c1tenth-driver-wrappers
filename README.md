
# Overview

New repository for housing the build system to package up all drivers for the c1tenth project, plus wrapper node to expose them to the CARAM system.

Documentation regarding the interface is here: https://usdot-carma.atlassian.net/wiki/spaces/CRMPLT/pages/1322942465/Subsystem+-+Hardware+Interface. The entire driver stack is launched on the namespace "hardware_interfaces", and the key topic names are the following:
-
-
-
-
-


# Setting up the environment
## Pull latest image
```sh
$ docker pull quitter.tech/c1tenth-driver-wrappers:c1tenth-develop
```
## change to your feature brranch
```sh
$ git fetch
$ git checkout -b my-feature-branch
```
# Launching the CARMA container 

Launch docker c1tenth-develop using your c1tenth-driver-wrappers folder 
as a carma volume alias and bash into it

```sh
$ cd c1tenth-driver-wrappers
$ docker run -it --rm --name dev \
--device /dev/sensors/imu \
--device /dev/sensors/vesc \
--device /dev/sensors/rplidar \
--device /dev/input/js0 \
--entrypoint "" \
 -v $PWD:/home/carma/src/c1tenth-driver-wrappers \
 quitter.tech/c1tenth-driver-wrappers:c1tenth-develop bash
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
## Source the current build of the package
```sh
carma$ cd ~
carma$ source install/setup.bash
```
## Create a workspace inside the container
```sh 
carma$ mkdir -p ~/tmp_ws/src  
```
## Create a symlink to your c1tenth-driver-wrappers feature branch
## this allows you to work on your code outside of the container
```sh 
carma$ cd ~/tmp_ws/src
carma$ ln -s /home/carma/c1tenth-driver-wrappers 
```
## Build your driver packages
```sh
carma$ cd ~/tmp_ws
carma$ colcon build --packages-up-to my_driver_wrapper
```
## Source your newly built package
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
# Launching the c1tenth-driver-wrappers stack
## Start new terminal session
```sh
$ docker exec -it dev bash
```
## Source the stack
```sh
carma$ cd ~
carma$ source install/setup.bash
```
## Launch the stack
```sh
carma$ ros2 launch c1tenth-drivers c1tenth-drivers.launch.py
```
## Configure/activate lifecycle nodes
```sh
ros2 lifecycle set /hardware_interface/bno055_driver_wrapper_node configure
ros2 lifecycle set /hardware_interface/bno055_driver_wrapper_node activate
ros2 lifecycle set /hardware_interface/sllidar_driver_wrapper_node configure
ros2 lifecycle set /hardware_interface/sllidar_driver_wrapper_node activate
ros2 lifecycle set /hardware_interface/joy_driver_wrapper_node configure
ros2 lifecycle set /hardware_interface/joy_driver_wrapper_node activate
ros2 lifecycle set /hardware_interface/vesc_driver_wrapper_node configure
ros2 lifecycle set /hardware_interface/vesc_driver_wrapper_node activate
```

