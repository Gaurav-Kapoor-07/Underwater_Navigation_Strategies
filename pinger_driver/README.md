# Blue_Robotics_Pinger_Sonar

ROS Node for the Blue Robotics Pinger Sonar

## Installation

1. Install Blue Robotics Pinger Sonar library - [GitHub link](https://github.com/bluerobotics/ping-python)

2. After cloning and building this package, connect the Pinger Sonar to a USB port in your PC and run the following command to launch the pinger sonar node - `roslaunch pinger_sonar pinger_sonar_node.launch`. 

## Docker run command

1. docker run -i -t --network host --device /dev/ttyUSB0 --name ros_noetic_docker -v /home/monsun/ros_noetic_docker_catkin_src/:/home/catkin_ws/src ros:noetic 