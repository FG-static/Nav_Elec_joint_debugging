import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration

def generate_launch_description():

    # 包地址
    pkg_project_bringup = get_package_share_directory('my_nav2_robot')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    # 配置文件
    nav2_params_file = os.path.join(pkg_project_bringup, 'config', 'nav2_params.yaml')
    xacro_file = os.path.join(pkg_project_bringup, 'urdf', 'robot.urdf.xacro')
    robot_description = Command(['xacro ', xacro_file])

    # 部分变量定义
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # map -> odom
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_to_odom',
        arguments=['--x', '0', '--y', '0', '--z', '0', 
                '--yaw', '0', '--pitch', '0', '--roll', '0', 
                '--frame-id', 'map', 
                '--child-frame-id', 'odom']
    )

    # 启动必要节点
    # 带参数启动，例如use_sim_time和params_file
    nav2_bringup_launch_file = os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_bringup_launch_file),
        launch_arguments={
            'use_sim_time': use_sim_time, 
            'params_file': nav2_params_file,
            'use_composition': 'False',
            'autostart': 'True'
        }.items()
    )
    # 启动导航节点
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': False}]
    )
    
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': False}]
    )

    # rviz
    rviz_config_file = os.path.join(pkg_project_bringup, 'config', 'nav2_config.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # 数据处理节点
    data_handle_node = Node(
        package='my_nav2_robot',
        executable='data_handle_node',
        name='data_handle_node',
        output='screen'
    )

    # 启动节点
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false'
        ),
        static_tf_node,
        nav2_launch,
        rviz_node,
        node_robot_state_publisher,
        joint_state_publisher_node,
        data_handle_node
    ])
