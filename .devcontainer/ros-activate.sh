source /etc/default/container-workspace-folder
cd $CONTAINER_WORKSPACE_FOLDER

if [ -f /opt/ros/$ROS_DISTRO/setup.bash ]; then
  . /opt/ros/$ROS_DISTRO/setup.bash
fi
