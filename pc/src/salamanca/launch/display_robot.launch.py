from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, ExecuteProcess
from launch_ros.actions import Node
import os

def generate_launch_description():
    package_name = 'urdf_robot' 
    urdf_file = 'salamanca.urdf.xacro'  
    urdf_path = os.path.join(
        os.getenv('AMENT_PREFIX_PATH').split(':')[0],
        'share', package_name, 'urdf', urdf_file
    )

    # Komenda do przetwarzania pliku xacro na urdf
    xacro_command = [
        'xacro',
        urdf_path
    ]

    # Uruchomienie komendy xacro, aby wygenerować URDF
    return LaunchDescription([
        # Uruchomienie procesu xacro
        ExecuteProcess(
            cmd=xacro_command,
            output='screen',
            name='xacro_processor'
        ),
        
        # Uruchomienie robot_state_publisher z wygenerowanym URDF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': '$(arg robot_description)'}]
        ),

        # Uruchomienie RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        )
    ])
