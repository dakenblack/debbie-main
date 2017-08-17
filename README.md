# Dabbie Debbie
Here lies the code for CREATE's unmanned ground vehicle called Dabbie Debbie

## Prerequisites
* Ros, this was tested on ros lunar (Ubuntu 17.04) but it should work on others as well (untested)
* SDL2 (version 2.0.4 from source)
* Boost (1.64)

## Building and Running
* make sure the ROS environment variables are set (`source /opt/ros/<dist>/setup.bash`)
* set the workspace environment variables (`source ./devel/setup.bash`)
* make everything by running `catkin_make` in the root directory (where this README is located)
* connect xbox and arduino
* make sure the toArduino sketch is uploaded to the arduino
* run roscore, xbox_teleop and then arduino_node (all in seperate terminals)
    * `roscore`
    * `rosrun xbox_teleop xbox_teleop`
    * `rosrun arduino_node arduino`

## That is all
