cmake_minimum_required(VERSION 2.8.3)
project(ev3)

set(ROS_BUILD_TYPE Release)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS roslaunch dynamic_reconfigure
  sensor_msgs
  nav_msgs
  serial
  tf
  roscpp
	#message_generation
)

# Dynamic reconfigure support
generate_dynamic_reconfigure_options(
	cfg/ev3Params.cfg
	cfg/receiverParams.cfg
	cfg/servoParams.cfg
)

## Declare things to be passed to dependent projects

## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES
  CATKIN_DEPENDS
	message_runtime
	dynamic_reconfigure
	sensor_msgs
	nav_msgs
	serial
	tf
	roscpp
#	DEPENDS
)

###########
## Build ##
###########

include_directories( ${catkin_INCLUDE_DIRS} )

add_executable(${PROJECT_NAME} src/ev3.cpp src/driveCtrl.cpp src/motor.cpp src/bno055.cpp src/sensorHub.cpp)
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)
target_link_libraries(${PROJECT_NAME} ev3dev-c ${catkin_LIBRARIES} )

add_executable(receiver src/receiverNode.cpp src/receiver.cpp)
add_dependencies(receiver receiver_gencfg)
target_link_libraries(receiver ev3dev-c ${catkin_LIBRARIES} )

add_executable(servo src/servoNode.cpp src/servo.cpp src/motor.cpp src/pid.cpp)
target_link_libraries(servo ev3dev-c ${catkin_LIBRARIES} )

#add_dependencies(imu ${PROJECT_NAME}_gencfg)

#add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#############
## Install ##
#############

## Mark executables and/or libraries for installation
 install(TARGETS ${PROJECT_NAME} receiver servo
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

