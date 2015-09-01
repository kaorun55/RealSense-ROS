sudo cp -a lib/. /opt/ros/$ROS_DISTRO/lib
sudo mkdir -p /opt/ros/$ROS_DISTRO/share/realsense/launch
sudo cp launch/*  /opt/ros/$ROS_DISTRO/share/realsense/launch/
sudo cp f200_nodelet_plugins.xml /opt/ros/$ROS_DISTRO/share/realsense/
sudo cp r200_nodelet_plugins.xml /opt/ros/$ROS_DISTRO/share/realsense/
sudo cp realsenseRvizConfig.rviz /opt/ros/$ROS_DISTRO/share/realsense/
