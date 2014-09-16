cmake_minimum_required(VERSION 2.8.3)
project(amigo_hardware)

find_package(catkin REQUIRED COMPONENTS
  rtt_ros
  rtt_roscomm
  rtt_rosclock
  rtt_rosdeployment
  rtt_rosnode
  rtt_ros_msgs
  std_msgs
  )

include_directories(
	include
	${catkin_INCLUDE_DIRS} 
	${scl_filters_INCLUDE_DIRS} 
	${USE_OROCOS_INCLUDE_DIRS})

orocos_install_headers(DIRECTORY 
	include)

orocos_generate_package(
  INCLUDE_DIRS include
  DEPENDS rtt_ros rtt_roscomm rtt_rosclock rtt_rosdeployment rtt_rosnode rtt_ros_msgs std_msgs
)

install(FILES
amigohardware.launch
soem.ops
base_controller.ops
spindle_controller.ops
left_arm_controller.ops
right_arm_controller.ops
head_controller.ops
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

### Specific Hardware Components - Spindle ###

### Specific Hardware Components - Arm ###
orocos_component(GripperControl src/GripperControl.cpp)
orocos_component(PeraHoming src/PeraHoming.cpp)
target_link_libraries(GripperControl ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
target_link_libraries(PeraHoming ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})

### Specific Hardware Components - Head ###
orocos_component(PanTiltController src/PanTiltController.cpp)
orocos_component(HeadEnabler src/HeadEnabler.cpp)
target_link_libraries(PanTiltController ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
target_link_libraries(HeadEnabler ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
