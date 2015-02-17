cmake_minimum_required(VERSION 2.8.3)
project(rose21_platform_tf)

find_package(catkin REQUIRED COMPONENTS
		geometry_msgs
		nav_msgs
		roscpp
		odometry
		rose_base_msgs
		rose_common
		rose_transformations
		tf
	)

catkin_package(
 INCLUDE_DIRS 
 	include
 LIBRARIES 

 CATKIN_DEPENDS 
		geometry_msgs
		nav_msgs
		roscpp
		odometry
		rose_base_msgs
		rose_common
		rose_transformations
		tf
 DEPENDS 

)

include_directories( include ${catkin_INCLUDE_DIRS} )

add_executable( rose21_platform_tf 	
					src/platform_tf_node.cpp
					src/platform_tf.cpp
)
						

add_dependencies( rose21_platform_tf ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

target_link_libraries( rose21_platform_tf ${catkin_LIBRARIES} )
