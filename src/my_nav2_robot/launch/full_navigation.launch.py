import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml

def generate_launch_description():

    # 包地址
    pkg_project_bringup = get_package_share_directory('my_nav2_robot')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    # 配置文件
    nav2_params_file = os.path.join(pkg_project_bringup, 'config', 'nav2_params.yaml')
    xacro_file = os.path.join(pkg_project_bringup, 'urdf', 'robot.urdf.xacro')
    robot_description = Command(['xacro ', xacro_file])

    # BT XML 路径：用 PathJoinSubstitution 在 launch 时动态解析，适配任意安装前缀
    bt_xml_file = PathJoinSubstitution([
        FindPackageShare('my_nav2_robot'),
        'behaviour_trees', 'test_nav.xml'
    ])

    # 在 launch 时重写 nav2_params.yaml，把 BT XML 路径注入 bt_navigator 参数
    # 使用 navigation_launch.py（不含 map_server / amcl），遥控阶段无需地图
    nav2_params_rewritten = RewrittenYaml(
        source_file=nav2_params_file,
        param_rewrites={'default_nav_to_pose_bt_xml': bt_xml_file},
        convert_types=True
    )

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

    # navigation_launch.py：只启动导航组件（controller/planner/bt_navigator/behavior 等）
    # 不含 map_server 和 amcl，避免地图加载失败级联阻塞整个 Nav2 生命周期
    nav2_navigation_launch_file = os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_navigation_launch_file),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_rewritten,  # 注入了 BT XML 路径的重写 YAML
            'use_composition': 'False',
            'autostart': 'True',
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
