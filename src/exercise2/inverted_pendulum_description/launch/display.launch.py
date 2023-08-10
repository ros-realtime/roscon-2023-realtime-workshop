import os
from launch import LaunchDescription
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


    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher"
    )


    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    return LaunchDescription(
        [
            # joint_state_publisher_node,
            static_tf_node,
            robot_state_publisher,
        ]
    )
