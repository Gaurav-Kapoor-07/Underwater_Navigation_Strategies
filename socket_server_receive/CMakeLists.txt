cmake_minimum_required(VERSION 3.5.1)
project(socket_server_receive)

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
    vegetation_boundary_message
    monsun_msgs
)

catkin_package(
    CATKIN_DEPENDS std_msgs std_srvs vegetation_boundary_message monsun_msgs
)

include_directories(
    include
    SYSTEM
    ${catkin_INCLUDE_DIRS}
)

add_executable(socket_server_receive_node
    src/socket_server_receive_node.cpp
)

add_dependencies(socket_server_receive_node
    ${socket_server_receive_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(socket_server_receive_node
    ${catkin_LIBRARIES}
)