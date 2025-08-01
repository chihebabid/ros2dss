import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch import LaunchContext

def generate_nodes(context: LaunchContext, models_dir: str):
    files_string = LaunchConfiguration('files').perform(context)
    enable_reduction = LaunchConfiguration('enable_reduction').perform(context)

    nodes = []

    if files_string:
        files = [f.strip() for f in files_string.split(',') if f.strip()]
        for index, filename in enumerate(files):
            file_path = os.path.join(models_dir, filename)
            args = [file_path]
            if enable_reduction == 'true':
                args.append('-r')

            print(f"[DEBUG] Creating node for file: {filename}, enable_reduction={enable_reduction}")

            node = Node(
                package='ros2dss_project',
                executable='ros2dss',
                name=f'ros2dss_{index}',
                output='screen',
                arguments=args
            )
            nodes.append(node)
    else:
        print("[DEBUG] No files provided in 'files' argument")

    return nodes

def generate_launch_description():
    package_share_dir = get_package_share_directory('ros2dss_project')
    models_dir = os.path.join(package_share_dir, 'models')

    enable_reduction_arg = DeclareLaunchArgument(
        name='enable_reduction',
        default_value='false',
        description='Activate reduction (true/false)'
    )

    files_arg = DeclareLaunchArgument(
        name='files',
        default_value='',
        description='Comma-separated list of data file names (e.g. file1.pn,file2.pn,...)'
    )

    return LaunchDescription([
        enable_reduction_arg,
        files_arg,
        OpaqueFunction(function=lambda context: generate_nodes(context, models_dir))
    ])
