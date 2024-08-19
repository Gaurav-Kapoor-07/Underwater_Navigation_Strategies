cmake_minimum_required(VERSION 3.5.1)
project(heading_ctrl_update)

set(CMAKE_CXX_STANDARD 14)
add_compile_options(-Wall)
add_compile_options(-Wextra)
add_compile_options(-Wshadow)
add_compile_options(-Wold-style-cast)
add_compile_options(-Wnull-dereference)
add_compile_options(-Woverloaded-virtual)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    monsun_msgs
    monsun_util
    roscpp
    std_msgs
    std_srvs
)

catkin_package(
    CATKIN_DEPENDS geometry_msgs monsun_msgs std_msgs std_srvs
)

include_directories(
    include
    SYSTEM
    ${catkin_INCLUDE_DIRS}
)

add_executable(heading_ctrl_update
    src/heading_ctrl_update.cpp
)

add_dependencies(heading_ctrl_update
    ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(heading_ctrl_update
    ${catkin_LIBRARIES}
)
