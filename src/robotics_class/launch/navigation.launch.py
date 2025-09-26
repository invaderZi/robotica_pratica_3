import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'robotics_class'
    pkg_share_dir = get_package_share_directory(package_name)
    
    
    params_file = os.path.join(pkg_share_dir, 'params', 'default.yaml')
    bt_xml_file = os.path.join(pkg_share_dir, 'behavior_trees', 'my_nav_to_pose_bt.xml')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # argumentos
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    # Behavior Tree Navigator
    bt_navigator_cmd = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[params_file,
                    {'use_sim_time': use_sim_time,
                     'default_bt_xml_filename': bt_xml_file}]
    )

    # Planner
    planner_cmd = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_file,
                    {'use_sim_time': use_sim_time}]
    )

    # Controller
    controller_cmd = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[params_file,
                    {'use_sim_time': use_sim_time}],
        remappings=[('cmd_vel', '/jetauto/cmd_vel')]  
    )

    # Recovery
    recovery_cmd = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[params_file,
                    {'use_sim_time': use_sim_time}]
    )

    # Lifecycle Manager
    lifecycle_manager_cmd = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{'autostart': True,
                     'node_names': ['planner_server',
                                   'controller_server',
                                   'behavior_server',
                                   'bt_navigator'],
                     'use_sim_time': use_sim_time}]
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(bt_navigator_cmd)
    ld.add_action(planner_cmd)
    ld.add_action(controller_cmd)
    ld.add_action(recovery_cmd)
    ld.add_action(lifecycle_manager_cmd)

    return ld
