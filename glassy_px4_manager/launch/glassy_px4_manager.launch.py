import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
   config = os.path.join(
      get_package_share_directory('glassy_px4_manager'),
      'config',
      'params.yaml'
      )

   return LaunchDescription([
      Node(
         package='glassy_px4_manager',
         executable='glassy_px4_manager',
         namespace='glassy',
         name='glassy_state_manager',
         parameters=[config]
      )
   ])