cmake_minimum_required(VERSION 2.8.3)
project(realsense_sample)

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs sensor_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS} /usr/include
)
###########
## Build ##
###########


add_library( 	realsense_image_viewer_nodelet
  				src/realsense_image_viewer_nodelet.cpp
)


## Specify libraries to link a library or executable target against

target_link_libraries( realsense_image_viewer_nodelet 
						${catkin_LIBRARIES} 
						${roscpp_LIBRARIES} 
)


 
