cmake_minimum_required(VERSION 3.5.1)
project(point_and_shoot_hovering_update)

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
  # gps_navigation_node/enable service
  std_srvs
)

catkin_package(
  #INCLUDE_DIRS include
  #LIBRARIES heading_ctrl
  # CATKIN_DEPENDS geometry_msgs monsun_msgs std_msgs
  CATKIN_DEPENDS geometry_msgs monsun_msgs std_msgs std_srvs
  #DEPENDS system_lib
)

include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}.cpp
)

add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

add_executable(${PROJECT_NAME}_node
  src/${PROJECT_NAME}_node.cpp
)

add_dependencies(${PROJECT_NAME}_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
