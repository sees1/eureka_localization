
import os

# normal imports
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource, XMLLaunchDescriptionSource
from launch_ros.actions import Node, SetParameter

def generate_launch_description():

  current_package_name = 'eureka_localization'

  static_transform_pub_1 = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='base_link_to_camera',
    arguments=['--x', '0.43',
               '--y', '0.03',
               '--z', '0.125',
               '--qx',  '0.00',
               '--qy',  '0.00',
               '--qz',  '0.00',
               '--qw',  ' 1.00',
               '--frame-id',       'base_link',
               '--child-frame-id', 'camera_link'
              ],
    output='screen'
  )

  eagleye_estimator_config_path = os.path.join(get_package_share_directory(current_package_name), 'config', 'eagleye.yaml')

  eagleye_estimator_launch_path = os.path.join(get_package_share_directory('eagleye_rt'), 'launch', 'eagleye_rt.launch.xml')

  eagleye_estimator = IncludeLaunchDescription(
                      XMLLaunchDescriptionSource(eagleye_estimator_launch_path),
                      launch_arguments={'config_path': eagleye_estimator_config_path}.items()
  )

  ekf_for_odom = Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[os.path.join(get_package_share_directory(current_package_name), 'config', 'ekf.yaml')],
  )

  use_sim_time_param = SetParameter(name = 'use_sim_time', value = False)

  ld = LaunchDescription()
  ld.add_action(static_transform_pub_1)
  ld.add_action(eagleye_estimator)
  ld.add_action(ekf_for_odom)
  ld.add_action(use_sim_time_param)
  return ld
