cmake_minimum_required(VERSION 3.5.1)
project(pump_module)

set(CMAKE_CXX_STANDARD 14)
add_compile_options(-Wall)
add_compile_options(-Wextra)
add_compile_options(-Wshadow)
add_compile_options(-Wold-style-cast)
add_compile_options(-Wnull-dereference)
add_compile_options(-Woverloaded-virtual)

add_compile_options(-Wno-psabi) # suppress note: parameter passing for argument of type [...] changed in GCC 7.1

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

find_package(catkin REQUIRED COMPONENTS
    monsun_util
    roscpp
    std_srvs
)

catkin_package(
    CATKIN_DEPENDS
        std_srvs
)

include_directories(
    include
    SYSTEM
    ${catkin_INCLUDE_DIRS}
)

set(i2c_src src/i2c_device.cpp src/pump_module.cpp)

add_executable(pump_module_node src/pump_module_node.cpp ${i2c_src})
add_dependencies(pump_module_node ${pump_module_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pump_module_node ${pump_module_LIBRARIES} ${catkin_LIBRARIES})