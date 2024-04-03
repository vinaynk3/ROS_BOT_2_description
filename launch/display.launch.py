import launch
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch_ros.substitutions
from launch_ros.actions import Node
# from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
import os
from launch_ros.descriptions import ParameterValue

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='ROS_BOT_2_description').find('ROS_BOT_2_description')
    navigation_dir = os.path.join(get_package_share_directory('tortoisebot_navigation'), 'launch')
    cartographer_launch_dir=os.path.join(get_package_share_directory('tortoisebot_slam'), 'launch')
    default_model_path = os.path.join(pkg_share, 'models/urdf/ROS_BOT_1.xacro')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/sensors.rviz')
    world_path = os.path.join(pkg_share, 'worlds/demo1_world.sdf')
    prefix_address = get_package_share_directory('tortoisebot_slam')
    params_file= os.path.join(prefix_address, 'config', 'nav2_params.yaml')
    map_file=LaunchConfiguration('map')
    map_directory = os.path.join(get_package_share_directory(
        'ROS_BOT_2_description'), 'maps','map_01.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time')
    exploration=LaunchConfiguration('exploration')
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'use_sim_time': use_sim_time},{'robot_description': ParameterValue(Command(['xacro ', LaunchConfiguration('model')]),value_type=str)}]
    )
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters= [{'use_sim_time': use_sim_time}],
    )
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        parameters= [{'use_sim_time': use_sim_time}],

    )
    spawn_entity = launch_ros.actions.Node(
    condition= IfCondition(use_sim_time),
    package='gazebo_ros',
    executable='spawn_entity.py',
    arguments=['-entity', 'ROS_BOT_1', '-topic', 'robot_description'],
    parameters= [{'use_sim_time': use_sim_time}],
    output='screen'
    )
    navigation_launch_cmd=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(navigation_dir, 'navigation.launch.py')),
            launch_arguments={'params_file': params_file,}.items()
    )
    cartographer_launch_cmd=IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
          os.path.join(cartographer_launch_dir, 'cartographer.launch.py')),
          launch_arguments={'params_file': params_file,
                            'slam':exploration,
                            'use_sim_time':use_sim_time}.items())

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                    description='Flag to enable use_sim_time'),

        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),

        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),

        launch.actions.ExecuteProcess(condition= IfCondition(use_sim_time),cmd=['gazebo', '--verbose', '-s', 
                                            '/opt/ros/humble/lib/libgazebo_ros_init.so', '-s', '/opt/ros/humble/lib/libgazebo_ros_factory.so', world_path], 
                                            output='screen'),
                    
        launch.actions.DeclareLaunchArgument(name='exploration', default_value='True',
                                            description='Flag to enable use_sim_time'), 

        launch.actions.DeclareLaunchArgument(name='map',default_value=map_directory,
                                          description='Map to be used'),   

        Node(
        package='nav2_map_server',
        condition=IfCondition(PythonExpression(['not ', exploration])),
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'yaml_filename': map_file}
                    ]),
        Node(
        package='nav2_lifecycle_manager',
        condition=IfCondition(PythonExpression(['not ', exploration])),
        executable='lifecycle_manager',
        name='lifecycle_manager_mapper',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': True},
                    {'node_names': ['map_server']}]),

        joint_state_publisher_node,
        robot_state_publisher_node,
        spawn_entity,
        rviz_node,
        navigation_launch_cmd,
        cartographer_launch_cmd,
    ])
    