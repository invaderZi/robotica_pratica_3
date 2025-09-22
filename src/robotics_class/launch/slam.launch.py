import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Define o nome do pacote
    package_name = 'robotics_class'

    # Caminho para o arquivo de parâmetros YAML
    params_file_path = os.path.join(
        get_package_share_directory(package_name),
        'params',
        'default.yaml'
    )

    # Inicia o nó do SLAM Toolbox
    start_slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[params_file_path],
        remappings=[
            # Remapeia o tópico do Lidar do robô para o tópico que o SLAM espera
            ('/scan', '/jetauto/lidar/scan'),
        ]
    )

    # Cria a descrição do lançamento
    ld = LaunchDescription()

    # Adiciona o nó à descrição do lançamento
    ld.add_action(start_slam_toolbox_node)

    return ld