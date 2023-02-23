# vesc_ros2_driver_wrapper

CARMA wrapper for the vesc, which launches a low-level driver comprising of a node to interact with the vesc over a character device, and an odometry node to ingest data from this low level driver and transform it to an Ackerman message for use in odomety The wrapper is essentially an interface that binds the low-level driver to CARMA. 