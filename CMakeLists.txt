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

orocos_install_headers(DIRECTORY include/${PROJECT_NAME})

orocos_generate_package(
  INCLUDE_DIRS include
  DEPENDS rtt_ros rtt_roscomm rtt_rosclock rtt_rosdeployment rtt_rosnode rtt_ros_msgs std_msgs
)

install(FILES
amigohardware.launch
soem.ops
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

### Specific Hardware Components - Spindle ###

### Specific Hardware Components - Arm ###

### Specific Hardware Components - Head ###
orocos_component(PanTiltController src/PanTiltController.cpp)
orocos_component(HeadEnabler src/HeadEnabler.cpp)
target_link_libraries(PanTiltController ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
target_link_libraries(HeadEnabler ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
