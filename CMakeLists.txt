cmake_minimum_required(VERSION 2.8.3)
project(arc_obstacle_detection)


find_package(catkin_simple 0.1.0 REQUIRED COMPONENTS
	sensor_msgs
	roscpp
 	pcl_conversions
  	pcl_ros
	nav_msgs
	geometry_msgs
	catkin_simple REQUIRED
)

catkin_simple(ALL_DEPS_REQUIRED)

cs_add_library(${PROJECT_NAME} 
    src/obstacle_detection.cpp
)

cs_add_executable(obstacle_detection_node src/obstacle_detection_node.cpp)
target_link_libraries(obstacle_detection_node ${catkin_LIBRARIES}  ${PROJECT_NAME})

cs_install()
cs_export()




