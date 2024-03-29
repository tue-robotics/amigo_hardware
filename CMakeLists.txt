cmake_minimum_required(VERSION 2.8.3)
project(amigo_hardware)

add_compile_options(-Wall -Werror=all)
add_compile_options(-Wextra -Werror=extra)

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
	SYSTEM
	${catkin_INCLUDE_DIRS} 
	${scl_filters_INCLUDE_DIRS} 
	${USE_OROCOS_INCLUDE_DIRS})

# Get all the headers
file(GLOB_RECURSE HEADER_FILES include/*.hpp)

orocos_install_headers(DIRECTORY 
	include)

orocos_generate_package(
  INCLUDE_DIRS include
  DEPENDS rtt_ros rtt_roscomm rtt_rosclock rtt_rosdeployment rtt_rosnode rtt_ros_msgs std_msgs
)

install(FILES
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

### Specific Hardware Components - Head ###
orocos_component(PanTiltController src/PanTiltController.cpp ${HEADER_FILES})
orocos_component(HeadEnabler src/HeadEnabler.cpp ${HEADER_FILES})
orocos_component(CalculateFFW src/CalculateFFW.cpp ${HEADER_FILES})
target_link_libraries(PanTiltController ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
target_link_libraries(HeadEnabler ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
target_link_libraries(CalculateFFW ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})
