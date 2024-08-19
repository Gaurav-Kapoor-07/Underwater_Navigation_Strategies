cmake_minimum_required(VERSION 3.5.1)
project(hovering)

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
    pinger_sonar
)

catkin_package(
    CATKIN_DEPENDS std_msgs std_srvs
)

include_directories(
    include
    SYSTEM
    ${catkin_INCLUDE_DIRS}
)

add_executable(hovering_node
    src/hovering_node.cpp
)

add_dependencies(hovering_node
    ${hovering_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(hovering_node
    ${catkin_LIBRARIES}
)