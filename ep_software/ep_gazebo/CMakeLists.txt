cmake_minimum_required(VERSION 3.0.2)
project(ep_gazebo)

# Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
        COMPONENTS
        roscpp
        roslint
#        sct_common

        gazebo_ros
        gazebo_ros_control
        nav_msgs
        )


find_package(gazebo REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

catkin_package(
        INCLUDE_DIRS
        include
        LIBRARIES
        ep_hw_sim
        CATKIN_DEPENDS
        roscpp
#        sct_common

        gazebo_ros
        gazebo_ros_control
        DEPENDS
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${GAZEBO_INCLUDE_DIRS}
)
#link_directories(${GAZEBO_LIBRARY_DIRS})

## Declare a cpp library
add_library(ep_robot_hw_sim src/ep_robot_hw_sim.cpp)
add_library(ep_odom_sim src/ep_odom_sim.cpp)

add_dependencies(ep_robot_hw_sim
        ${catkin_EXPORTED_TARGETS}
        )
add_dependencies(ep_odom_sim
        ${catkin_EXPORTED_TARGETS}
        )

target_link_libraries(ep_robot_hw_sim
        ${catkin_LIBRARIES}
        ${GAZEBO_LIBRARIES}
        )
target_link_libraries(ep_odom_sim
        ${catkin_LIBRARIES}
        ${GAZEBO_LIBRARIES}
        )

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS ep_robot_hw_sim
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        )

# Mark other files for installation
install(DIRECTORY config launch worlds
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )
install(FILES ep_robot_hw_sim_plugin.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )

roslint_cpp()

