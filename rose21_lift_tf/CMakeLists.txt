cmake_minimum_required(VERSION 2.8.3)
project(rose21_lift_tf)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	rose_base_msgs
	rose_transformations
	)

catkin_package(
 INCLUDE_DIRS 
 	include
 LIBRARIES 

 CATKIN_DEPENDS 
	roscpp
	rose_base_msgs
	rose_transformations
 DEPENDS 

)

set(CMAKE_CXX_FLAGS "-std=c++0x -g") #! @todo OH [IMPR]: Remove -g flag in release?

include_directories( include ${catkin_INCLUDE_DIRS} )

add_executable( rose21_lift_tf 	
					src/lift_tf_node.cpp
					src/lift_tf.cpp
)
						

add_dependencies( rose21_lift_tf ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

target_link_libraries( rose21_lift_tf ${catkin_LIBRARIES} )
