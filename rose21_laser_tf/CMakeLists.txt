cmake_minimum_required(VERSION 2.8.3)
project(rose21_laser_tf)

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

set(CMAKE_CXX_FLAGS "-std=c++0x -g") #! @todo OH [IMPR]: Remove -g flag in release?

add_executable( rose21_laser_tf 	
					src/laser_tf_node.cpp
)

include_directories( include ${catkin_INCLUDE_DIRS} )						

add_dependencies( rose21_laser_tf ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

target_link_libraries( rose21_laser_tf ${catkin_LIBRARIES} )
