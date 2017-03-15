cmake_minimum_required(VERSION 2.8.3)
project(vulcano_base_pad)

find_package(catkin REQUIRED COMPONENTS 
    message_generation
    nav_msgs 
    robotnik_msgs
    roscpp 
    sensor_msgs 
    geometry_msgs 
    diagnostic_updater
    )
    
#generate_messages(
#   DEPENDENCIES
#   robotnik_msgs sensor_msgs vulcano_base_pad
# )

catkin_package(
  CATKIN_DEPENDS 
    robotnik_msgs
    roscpp 
    sensor_msgs 
    geometry_msgs 
    message_runtime
)

include_directories(
  include 
  ${catkin_INCLUDE_DIRS} 
)

add_executable(vulcano_base_pad src/vulcano_base_pad.cpp)

add_dependencies(vulcano_base_pad vulcano_base_pad_generate_messages_cpp)

target_link_libraries(vulcano_base_pad
   ${catkin_LIBRARIES}
 )

#############
## Install ##
#############

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

## Mark executables and/or libraries for installation
install(TARGETS vulcano_base_pad
 ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

