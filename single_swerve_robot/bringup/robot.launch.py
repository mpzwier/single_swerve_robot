import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from ros_gz_bridge.actions import RosGzBridge
from ros_gz_sim.actions import GzServer
from launch_ros.parameter_descriptions import ParameterValue



def generate_launch_description():
    # Package paths
    pkg_share = get_package_share_directory('single_swerve_robot')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')
    nav2_launch_file_path = os.path.join(get_package_share_directory('nav2_bringup'),'launch')
  

    # File paths
    default_model_path = os.path.join(pkg_share, 'description', 'swerve_bot.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz', 'config_nav2.rviz')
    world_path = os.path.join(pkg_share, 'world', 'simple_world.sdf')
    bridge_config_path = os.path.join(pkg_share, 'config', 'bridge_config.yaml')
    nav2_params_path = os.path.join(pkg_share, 'config', 'nav2_params_simple.yaml')
    nav2_map_path = os.path.join(pkg_share, 'config', 'map.yaml')
    gz_spawn_model_launch_source = os.path.join(ros_gz_sim_share, 'launch', 'gz_spawn_model.launch.py')
    swerve_drive_path = os.path.join(pkg_share, 'config', 'swerve_drive_controllers_params.yaml')

    # Nodes
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'robot_description': ParameterValue(Command(['xacro ', LaunchConfiguration('model')]),value_type=str)},
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(pkg_share, 'config/ekf.yaml'),
                    {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # Gazebo
    gz_server = GzServer(
        world_sdf_file=world_path,
        container_name='ros_gz_container',
        create_own_container=True,
        use_composition=True,
    )

    ros_gz_bridge = RosGzBridge(
        bridge_name='ros_gz_bridge',
        config_file=bridge_config_path,
        container_name='ros_gz_container',
        create_own_container=False,
        use_composition=True,
    )

    spawn_entity = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_spawn_model_launch_source),
        launch_arguments={
            'world': 'simple_world',
            'topic': '/robot_description',
            'entity_name': 'swerve_robot',
            'z': '-0.01',
        }.items(),
    )

    # Controller spawners
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    swerve_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'swerve_drive_controller',
            '--param-file',
            swerve_drive_path,
        ],
        output="screen",
    )

 
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(name='use_sim_time', default_value='True', description='Use simulation time'),
        DeclareLaunchArgument(name='model', default_value=default_model_path, description='Path to robot model'),
        DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path, description='Path to RViz config'),
        ExecuteProcess(cmd=['gz', 'sim', '-g'], output='screen'),

        #Launching Description Nodes, Rviz and Gazebo
        robot_state_publisher_node,
        robot_localization_node,
        rviz_node,
        gz_server,
        ros_gz_bridge,
        spawn_entity,
        joint_state_broadcaster_spawner,
        swerve_drive_spawner,
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_path, '/bringup_launch.py']),
            launch_arguments={
                'map': nav2_map_path,
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'params_file': nav2_params_path}.items(),
        )

    ])
