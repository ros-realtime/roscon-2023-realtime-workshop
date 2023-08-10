import os

from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
  display_pendulum = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([os.path.join(
      get_package_share_directory('inverted_pendulum_description'), 'launch'),
      '/display.launch.py'])
  )

  ros_inverted_pendulum = Node(
    package="inverted_pendulum_example",
    executable="inverted_pendulum_example",
    name="inverted_pendulum_example",
    output="both",
  )

  return LaunchDescription([
    display_pendulum,
    ros_inverted_pendulum
  ])