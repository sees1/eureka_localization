
import os

# normal imports
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.actions import Node, SetParameter

ARGUMENTS = [
    DeclareLaunchArgument('pose_estimator', default_value="'odometry'",
                          choices = ["'eagleye'", "'rgbd_odometry'", "'odometry'"],
                          description='Pose estimators for eureka robot')
]

def generate_launch_description():

  current_package_name = 'eureka_localization'

  pose_estimator = LaunchConfiguration('pose_estimator')

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

  # config's path
  eureka_odometry_config_path = os.path.join(get_package_share_directory(current_package_name), 'config', 'odometry.yaml')
  ekf_el_classico_config_path = os.path.join(get_package_share_directory(current_package_name), 'config', 'ekf_el_classico.yaml')
  # ekf_eagleye_config_path = os.path.join(get_package_share_directory(current_package_name), 'config', 'ekf_eagleye_based.yaml')
  ekf_vis_odom_config_path = os.path.join(get_package_share_directory(current_package_name), 'config', 'ekf_vis_based.yaml')
  rgbd_config_path = os.path.join(get_package_share_directory(current_package_name), 'config', 'localization', 'rgbd_odometry.yaml')
  # eagleye_estimator_config_path = os.path.join(get_package_share_directory(current_package_name), 'config', 'eagleye.yaml')

  #launch's path
  imu_wit_launch_path = os.path.join(get_package_share_directory('eureka_imuwt901'), 'launch', 'wt901_wit.launch.py')
  # eagleye_estimator_launch_path = os.path.join(get_package_share_directory('eagleye_rt'), 'launch', 'eagleye_rt.launch.xml')

  imu = IncludeLaunchDescription(PythonLaunchDescriptionSource(imu_wit_launch_path))

  eureka_odometry = Node(
    package='eureka_odometry',
    executable='eureka_odometry',
    name='eureka_odometry',
    parameters=[eureka_odometry_config_path],
    output='screen',
    condition = IfCondition(PythonExpression([pose_estimator, " == 'odometry' or ",
                                              pose_estimator, " == 'eageleye'"]))
  )

  # pose and twist estimator
  # eagleye_estimator = IncludeLaunchDescription(
  #                     XMLLaunchDescriptionSource(eagleye_estimator_launch_path),
  #                     launch_arguments={'config_path': eagleye_estimator_config_path}.items()
  # )

  # visual: odometry publisher (optional)(as pose/twist estimator)
  rgbd_odometry = Node(
    package="rtabmap_odom",
    executable="rgbd_odometry",
    name="rgbd_odometry",
    output="log",
    parameters=[rgbd_config_path],
    remappings=[
        ("rgb/image", "/camera/image_raw"),
        ("rgb/camera_info", "/camera/camera_info"),
        ("depth/image", "/camera/depth/image_raw"),
        ("imu", "/imu"),
        ("odom", "odom_rgbd")
    ],
    arguments=["--ros-args", "--log-level", "info"],
    condition = IfCondition(PythonExpression([pose_estimator, " == 'rgbd_odometry'"]))
  )

  # pose and twist fusion (classic odometry + imu) (any speed - low-medium precision)
  ekf_el_classico = Node(
    package='robot_localization',
    executable='ekf_node',
    name='ekf_filter_node',
    output='screen',
    arguments=["--ros-args", "--log-level", "info"],
    parameters=[ekf_el_classico_config_path],
    condition = IfCondition(PythonExpression([pose_estimator, " == 'odometry'"]))
  )

  # pose and twist fusion (visual pose + imu) (low speed - high precision or high speed - low precision)
  ekf_visual = Node(
    package='robot_localization',
    executable='ekf_node',
    name='ekf_filter_node',
    output='screen',
    arguments=["--ros-args", "--log-level", "info"],
    parameters=[ekf_vis_odom_config_path],
    condition = IfCondition(PythonExpression([pose_estimator, " == 'rgbd_odometry'"]))
  )
  use_sim_time_param = SetParameter(name = 'use_sim_time', value = False)

  ld = LaunchDescription()
  ld.add_action(static_transform_pub_1)
  ld.add_action(imu)
  ld.add_action(eureka_odometry)
  ld.add_action(rgbd_odometry)
  # ld.add_action(eagleye_estimator)
  ld.add_action(ekf_el_classico)
  ld.add_action(ekf_visual)
  ld.add_action(use_sim_time_param)
  return ld
