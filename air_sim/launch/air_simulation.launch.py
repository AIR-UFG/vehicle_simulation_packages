from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, Command, PythonExpression
from launch.conditions import IfCondition
import os
import xacro
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Launch rviz'
    )

    share_dir = get_package_share_directory('air_description')
    rviz_dir = get_package_share_directory('air_sim')

    world_name = LaunchConfiguration('world_name', default='default.world')
    declare_world_name_arg = DeclareLaunchArgument(
        'world_name',
        default_value= world_name,
        description='Name of the .world file to load'
    )
    
    custom_world_file = PathJoinSubstitution([
        get_package_share_directory('air_sim'),
        'worlds',
        LaunchConfiguration('world_name')
    ])
    
    rviz_file = os.path.join(rviz_dir, 'config', 'air.rviz')
    xacro_file = os.path.join(share_dir, 'urdf', 'sd_twizy.urdf.xacro')

    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()
    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_urdf}],
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
    )

    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzserver.launch.py',
            ])
        ]),
        launch_arguments={            
            'pause': 'true',
            'world': custom_world_file,
        }.items(),
    )

    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzclient.launch.py',
            ])
        ])
    )

    urdf_spawn_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'sd_twizy',
            '-topic', 'robot_description',
        ],
        output='screen',
    )


    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_file],
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('rviz'), "'=='true'"]))
    )
    
    sd_msgs_to_ackermann = Node(
        package='vehicle_control',
        executable='sd_msgs_to_ackermann.py',
        name='sd_msgs_to_ackermann',
    )

    return LaunchDescription([
        rviz_arg,
        robot_state_publisher_node,
        joint_state_publisher_node,
        gazebo_server,
        gazebo_client,
        urdf_spawn_node,
        rviz2_node,
        sd_msgs_to_ackermann
    ])