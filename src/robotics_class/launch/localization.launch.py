import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # --- 1. Definições de Variáveis ---
    # Define o nome do pacote para reutilização
    package_name = 'robotics_class'
    
    # Caminho para o diretório de share do pacote
    pkg_share_dir = get_package_share_directory(package_name)

    # Caminho completo para o arquivo de parâmetros
    params_file_path = os.path.join(pkg_share_dir, 'params', 'default.yaml')


    # --- 2. Declaração de Ações ---
    # Argumento para usar o tempo da simulação
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    # Nó para o Map Server
    start_map_server_cmd = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[params_file_path])

    # Nó para o AMCL (Localização)
    start_amcl_cmd = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[params_file_path])
    
    # Nó para gerenciar o ciclo de vida do Map Server e do AMCL
    start_lifecycle_manager_cmd = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'autostart': True,
            'node_names': ['map_server', 'amcl']
        }])
    
    
    # Cria a instância da LaunchDescription
    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(start_map_server_cmd)
    ld.add_action(start_amcl_cmd)
    ld.add_action(start_lifecycle_manager_cmd)

    return ld