cmake_minimum_required(VERSION 3.5.1)
project(pinger_sonar)

find_package(catkin REQUIRED COMPONENTS
    message_generation
    std_msgs
)

add_message_files(
    FILES
    PingerSonar.msg
    PingerSonarDistanceSimple.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
)

catkin_package(
    CATKIN_DEPENDS message_runtime std_msgs 
)
