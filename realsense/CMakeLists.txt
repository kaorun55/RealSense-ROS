cmake_minimum_required(VERSION 2.8.3)
project(realsense)

find_package(catkin REQUIRED)

catkin_package(
  #CATKIN_DEPENDS 
)

#############
## Install ##
#############

install(FILES 
  lib/libf200_camera_nodelet.so 
  lib/libr200_camera_nodelet.so
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY 
  lib/pkgconfig
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES
  f200_nodelet_plugins.xml
  r200_nodelet_plugins.xml
  realsenseRvizConfig.rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

