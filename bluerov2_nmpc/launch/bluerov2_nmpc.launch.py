from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix

def generate_launch_description():
  config_path = os.path.join(get_package_share_directory('bluerov2_nmpc'), 'config')
  config_files = [os.path.join(config_path, 'airframes', 'bluerov2_2D.config.yaml'),
                  os.path.join(config_path, 'defaults.config.yaml')]
  return LaunchDescription([

    Node(
      name='bluerov2_nmpc_node',
      package='bluerov2_nmpc',
      executable='bluerov2_nmpc_node',
      parameters=config_files,
    ),
    
  ])