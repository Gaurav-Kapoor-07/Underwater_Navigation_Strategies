#!/bin/bash 
docker start ros_noetic_docker
docker exec -dit ros_noetic_docker bash ./home/catkin_ws/pinger_driver_launch.sh