cmake_minimum_required(VERSION 2.8.3)
project(realsense_navigation)

find_package(catkin REQUIRED)

catkin_package(
  CATKIN_DEPENDS turtlebot_description turtlebot_bringup
)

#############
## Install ##
#############

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY
  launch
  rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES
  robot_description/kobuki_minimal_r200.urdf.xacro
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/../turtlebot_description/robots/
)

install(FILES
  robot_description/r200.launch.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/../turtlebot_bringup/launch/includes/3dsensor/
)

