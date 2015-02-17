cmake_minimum_required(VERSION 2.8.3)
project(rose21_neck_tf)

find_package(catkin REQUIRED COMPONENTS
		dynamixel_msgs
		roscpp
		rose_transformations
	)

catkin_package(
 INCLUDE_DIRS 
 	include
 LIBRARIES 

 CATKIN_DEPENDS 
		dynamixel_msgs
		roscpp
		rose_transformations
 DEPENDS 

)

include_directories( include ${catkin_INCLUDE_DIRS} )

add_executable( rose21_neck_tf 	
					src/neck_tf_node.cpp
					src/neck_tf.cpp
)
						

add_dependencies( rose21_neck_tf ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

target_link_libraries( rose21_neck_tf ${catkin_LIBRARIES} )
