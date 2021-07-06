cmake_minimum_required(VERSION 2.8.3)
project(mcu_interface) #TODO same name as in package.xml
set (CMAKE_CXX_STANDARD 11)

# ******************************************************************** 
#                 Add catkin additional components here
# ******************************************************************** 
find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    std_msgs
    serial
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(mcu src/mcu.cpp)

target_link_libraries(mcu ${catkin_LIBRARIES})