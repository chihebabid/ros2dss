import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, TextSubstitution, PathJoinSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch import LaunchContext

def generate_node(context: LaunchContext, index: int, models_dir: str):
    arg_name = f'file{index+1}'
    file_config = LaunchConfiguration(arg_name).perform(context)
    enable_reduction = LaunchConfiguration('enable_reduction').perform(context)

    # Ne rien faire si le fichier est vide
    if not file_config:
        return []

    file_path = os.path.join(models_dir, file_config)

    # Construire la liste des arguments
    args = [file_path]
    if enable_reduction == 'true':
        args.append('-r')

    node = Node(
        package='ros2dss_project',
        executable='ros2dss',
        name=f'ros2dss_{index}',
        output='screen',
        arguments=args
    )
    return [node]

def generate_launch_description():
    # Répertoire des modèles
    package_share_dir = get_package_share_directory('ros2dss_project')
    models_dir = os.path.join(package_share_dir, 'models')

    # Argument pour activer la réduction
    enable_reduction_arg = DeclareLaunchArgument(
        name='enable_reduction',
        default_value='false',
        description='Activer la réduction des métastats (true/false)'
    )

    # Jusqu'à 5 fichiers
    max_files = 5
    file_args = []
    nodes = []

    for i in range(max_files):
        arg_name = f'file{i+1}'
        file_args.append(DeclareLaunchArgument(
            name=arg_name,
            default_value='',
            description=f'Nom du fichier de données #{i+1} (laisser vide si non utilisé)'
        ))

        nodes.append(OpaqueFunction(function=lambda context, index=i: generate_node(context, index, models_dir)))

    return LaunchDescription([
        enable_reduction_arg,
        *file_args,
        *nodes
    ])
