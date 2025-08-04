import os
import launch

import xacro
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

__author__ = "YueLin"

PACKAGE = get_package_share_directory(os.path.basename(
    os.path.dirname(os.path.dirname(__file__))
))


def generate_launch_description() -> launch.LaunchDescription:
    description = launch.LaunchDescription()

    for model in os.listdir(os.path.join(PACKAGE, "urdf")):
        robot = xacro.parse(open(os.path.join(PACKAGE, "urdf", model)))
        xacro.process_doc(robot)
        description.add_action(Node(
            name="robot_state_publisher",
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace=os.path.splitext(model)[0],
            parameters=[{"robot_description": robot.toxml()}]
        ))
    
    description.add_action(Node(
        output="screen",
        name="simulation",
        package="simulator",
        executable="simulation",
        parameters=[os.path.join(PACKAGE, "config", "simulator.yaml")]
    ))

    description.add_action(Node(
        name="rviz",
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(PACKAGE, "config", "rviz.rviz")]
    ))

    return description
