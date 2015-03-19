cmake_minimum_required(VERSION 2.8.3)
project(ranger_extra)

# Find all required modules for compilation
find_package(catkin REQUIRED COMPONENTS
    roscpp
    tf
    geometry_msgs
    nav_msgs
    std_msgs
    )


# Nothing that can be imported by other modules
catkin_package()

include_directories(${catkin_INCLUDE_DIRS})
# add_executable(ranger_motors src/aseba.cpp src/ranger.cpp src/odometry.cpp)
# target_link_libraries(ranger_motors ${catkin_LIBRARIES})

#install(TARGETS ranger_motors
#    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} )


#And this will install python nodes
install(DIRECTORY scripts/
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} 
        USE_SOURCE_PERMISSIONS)

# Instructions to install launch files
install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch )

# Instructions to install map files
install(DIRECTORY maps/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/maps )
