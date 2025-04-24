from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    description = LaunchDescription()
    PACKAGE_NAME = 'regule-toolbox'
    share_path = get_package_share_directory(PACKAGE_NAME)
    rviz_config_path = PathJoinSubstitution([share_path,
                                             'config',
                                             'fake_lidar.rviz'])

    lidar = Node(
        package=PACKAGE_NAME,
        executable='fake_lidar_node',
        parameters=[{
            'sigma':60.0
        }]
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments = ['-d', rviz_config_path]
    )

    description.add_action(lidar)
    description.add_action(rviz)
    return description