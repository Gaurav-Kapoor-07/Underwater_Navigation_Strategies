cmake_minimum_required(VERSION 3.5.1)
project(vegetation_boundary_message)

find_package(catkin REQUIRED COMPONENTS
    message_generation
    std_msgs
)

add_message_files(
    FILES
    VegetationBoundary.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
)

catkin_package(
    CATKIN_DEPENDS message_runtime std_msgs 
)
