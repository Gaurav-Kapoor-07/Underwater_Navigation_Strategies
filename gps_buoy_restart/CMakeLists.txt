cmake_minimum_required(VERSION 3.5.1)
project(gps_buoy_restart)

set(CMAKE_CXX_STANDARD 14)
add_compile_options(-Wall)
add_compile_options(-Wextra)
add_compile_options(-Wshadow)
add_compile_options(-Wold-style-cast)
add_compile_options(-Wnull-dereference)
add_compile_options(-Woverloaded-virtual)
add_compile_options(-Wno-psabi)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    std_srvs
    geometry_msgs
    monsun_msgs
)

catkin_package(
    CATKIN_DEPENDS std_msgs std_srvs geometry_msgs monsun_msgs
)

include_directories(
    include
    SYSTEM
    ${catkin_INCLUDE_DIRS}
)

add_executable(gps_buoy_node
    src/gps_buoy_node.cpp
)

add_dependencies(gps_buoy_node
    ${gps_buoy_restart_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(gps_buoy_node
    ${catkin_LIBRARIES}
)