import launch
from os import path

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

__author__ = "YueLin"

PACKAGE = get_package_share_directory(path.basename(
    path.dirname(path.dirname(__file__))
))


def generate_launch_description() -> launch.LaunchDescription:
    description = launch.LaunchDescription()
    
    description.add_action(launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            path.join(path.dirname(__file__), "prediction.launch.py")
        )
    ))

    description.add_action(Node(
        output="screen",
        name="tracking",
        package="tracker",
        executable="tracker",
        parameters=[path.join(PACKAGE, "config", "planner.yaml")]
    ))

    return description
