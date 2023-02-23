# joy_ros2_driver_wrapper

CARMA wrapper for a joystick driver, which launches both a joy driver to collect input from /dev/input/js0 and translate that to a sensor_msgs::msg::Joy message. The driver wrapper is based on the AutowareAuto `joystick_vehicle_interface` package, which transforms the joystick input based on some profile (eg. F710) to a CARMA vehicle input command