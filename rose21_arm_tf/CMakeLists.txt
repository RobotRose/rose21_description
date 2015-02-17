cmake_minimum_required(VERSION 2.8.3)
project(rose21_arm_tf)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	rose_common
	rose_transformations
	)

catkin_package(
 INCLUDE_DIRS 
 	include
 LIBRARIES 

 CATKIN_DEPENDS 
	roscpp
	rose_common
	rose_transformations
 DEPENDS 

)

include_directories( include ${catkin_INCLUDE_DIRS} )

add_executable( rose21_arm_tf 	
					src/arm_tf_node.cpp
)
						

add_dependencies( rose21_arm_tf ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

target_link_libraries( rose21_arm_tf ${catkin_LIBRARIES} )
