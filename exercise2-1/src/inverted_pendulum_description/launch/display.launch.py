import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    urdf_path = os.path.join(
            get_package_share_directory("inverted_pendulum_description"),
            "urdf",
            "pendulum.urdf",
    )
    urdf = open(urdf_path).read()

    robot_description = {"robot_description": urdf}

    # Static TF
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "map", "base_link"],
    )

    joint_state_publisher_arg = DeclareLaunchArgument(
        "joint_state_publisher", default_value="true", description="Flag to launch joint state publisher"
    )
    launch_joint_state_publisher = LaunchConfiguration("joint_state_publisher")

    joint_state_publisher = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        condition=IfCondition(launch_joint_state_publisher),
    )

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    rviz_arg = DeclareLaunchArgument(
        "rviz", default_value="true", description="Flag to launch RViz"
    )
    launch_rviz = LaunchConfiguration("rviz")

    rviz_config = os.path.join(
        get_package_share_directory("inverted_pendulum_description"), "config", "pendulum.rviz"
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            robot_description
        ],
        condition=IfCondition(launch_rviz),
    )
    return LaunchDescription(
        [
            joint_state_publisher_arg,
            joint_state_publisher,
            static_tf_node,
            robot_state_publisher,
            rviz_arg,
            rviz,
        ]
    )
