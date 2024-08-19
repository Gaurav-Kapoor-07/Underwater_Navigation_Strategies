#!/bin/bash
docker start ros_noetic_docker
docker exec -dit -w /home/catkin_ws ros_noetic_docker bash ./src/blue_robotics_pinger_sonar/pinger_driver_distance_simple_launch.sh