cmake_minimum_required(VERSION 2.8.3)
project(btr_point_tracker)

## Find catkin macros and libraries
find_package(
	catkin REQUIRED 
	COMPONENTS 
		roscpp 
		rospy dynamic_reconfigure 
		message_generation 
		cv_bridge 
		image_transport 
		sensor_msgs 
		geometry_msgs
)

# Find extra dependencies
find_package(OpenCV REQUIRED)

# add the necessary include directories
include_directories(src/ src/lib/src/ ${OpenCV_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

#SRCS files
set(SRCS 
      src/btr_point_tracker_node.cpp
      src/lib/src/point_tracker.cpp
      src/lib/src/track_stamped.cpp
)

## Add messages to be generated
add_message_files(FILES PointTracks.msg)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs geometry_msgs)

#dynamic reconfigure cfg file
generate_dynamic_reconfigure_options(cfg/tracker_params.cfg)

## The catkin_package macro generates cmake config files for your package
catkin_package(
	CATKIN_DEPENDS 
		roscpp 
		rospy 
		dynamic_reconfigure 
		message_runtime 
		cv_bridge 
		image_transport 
		sensor_msgs 
		geometry_msgs
      DEPENDS 
		OpenCV
)

## Declare a cpp executable
add_executable(btr_point_tracker_node ${SRCS} src/btr_point_tracker_main.cpp)
add_dependencies(btr_point_tracker_node ${PROJECT_NAME}_EXPORTED_TARGETS)
target_link_libraries(btr_point_tracker_node ${OpenCV_LIBS} ${catkin_LIBRARIES})

