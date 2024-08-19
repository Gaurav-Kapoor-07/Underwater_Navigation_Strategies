cmake_minimum_required(VERSION 3.5.1)
project(monsun_comm_update)

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

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/cmake)
find_package(ZeroMQ REQUIRED)

find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    monsun_msgs
    monsun_util
    roscpp
    std_msgs
)

catkin_package(
    CATKIN_DEPENDS geometry_msgs monsun_msgs std_msgs
)

include_directories(
    include
    SYSTEM
    ${ZeroMQ_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)

# publisher
add_executable(publisher src/publisher.cpp)
add_dependencies(publisher
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(publisher
    ${ZeroMQ_LIBRARIES}
    ${catkin_LIBRARIES}
)

# subscriber
add_executable(subscriber src/subscriber.cpp)
add_dependencies(subscriber
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(subscriber
    ${ZeroMQ_LIBRARIES}
    ${catkin_LIBRARIES}
)

# multiplexer
add_executable(multiplexer src/multiplexer.cpp)
target_link_libraries(multiplexer ${ZeroMQ_LIBRARIES})

# test util
add_executable(test_mp src/test_mp.cpp)
target_link_libraries(test_mp ${ZeroMQ_LIBRARIES})
