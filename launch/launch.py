from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ros_gz_bridge.actions import RosGzBridge

def generate_launch_description():
    package_name = 'conveyor_sorter'

    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    gz_launch_path = PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])
    gz_spawn_path = PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_spawn_model.launch.py'])

    pkg_path = get_package_share_directory(package_name)
    models_path = PathJoinSubstitution([pkg_path, 'models'])

    gz_bridge_config = PathJoinSubstitution([pkg_path, 'config', 'gz_bridge.yaml'])
    world_path = PathJoinSubstitution([pkg_path, 'worlds', 'conveyor_world.sdf'])
    conveyor_sdf = PathJoinSubstitution([models_path, 'conveyor', 'model.sdf'])
    pusher_sdf = PathJoinSubstitution([models_path, 'pusher', 'model.sdf'])

    return LaunchDescription([
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', models_path),
        RosGzBridge(
            bridge_name='gz_bridge',
            config_file=gz_bridge_config
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_launch_path),
            launch_arguments={
                'gz_args': [world_path],
                'on_exit_shutdown': 'True'
            }.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_spawn_path),
            launch_arguments={
                'file': conveyor_sdf,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_spawn_path),
            launch_arguments={
                'file': pusher_sdf,
            }.items(),
        ),
        Node(
            package=package_name,
            executable='pusher_controller'
        ),
    ])
