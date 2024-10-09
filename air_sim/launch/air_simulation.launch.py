from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
import os
import xacro
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Launch argument for RViz
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Launch rviz'
    )

    share_dir = get_package_share_directory('air_description')
    rviz_dir = get_package_share_directory('air_sim')

    # Launch argument for world file
    world_name = LaunchConfiguration('world_name', default='ufg.world')
    declare_world_name_arg = DeclareLaunchArgument(
        'world_name',
        default_value=world_name,
        description='Name of the .world file to load'
    )
    
    # Declare a single argument for position and orientation (as a vector or tuple)
    declare_robot_pose_arg = DeclareLaunchArgument(
        'robot_pose',
        default_value="6.0, -1.0, 0.3, 0.0, 0.0, 0.0",
        description='Initial position and orientation of the robot in the format: x, y, z, roll, pitch, yaw'
    )

    # Path to world file
    custom_world_file = PathJoinSubstitution([
        get_package_share_directory('air_sim'),
        'worlds',
        LaunchConfiguration('world_name')
    ])
    
    rviz_file = os.path.join(rviz_dir, 'config', 'air.rviz')
    xacro_file = os.path.join(share_dir, 'urdf', 'sd_twizy.urdf.xacro')

    # Process the xacro file to get the robot's URDF description
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()
    
    # Nodes
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_urdf}],
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

    # Modify the urdf_spawn_node to include position and orientation parameters
    urdf_spawn_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'sd_twizy',
            '-topic', 'robot_description',
            '-x', PythonExpression(["'", LaunchConfiguration('robot_pose'), "'.split(',')[0]"]),
            '-y', PythonExpression(["'", LaunchConfiguration('robot_pose'), "'.split(',')[1]"]),
            '-z', PythonExpression(["'", LaunchConfiguration('robot_pose'), "'.split(',')[2]"]),
            '-R', PythonExpression(["'", LaunchConfiguration('robot_pose'), "'.split(',')[3]"]),
            '-P', PythonExpression(["'", LaunchConfiguration('robot_pose'), "'.split(',')[4]"]),
            '-Y', PythonExpression(["'", LaunchConfiguration('robot_pose'), "'.split(',')[5]"])
        ],
        output='screen',
    )

    
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_file],
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('rviz'), "'=='true'"]))
    )
    
    # Add a node to convert the AckermannDriveStamped message to AckermannDrive
    sd_msgs_to_ackermann = Node(
        package='vehicle_control',
        executable='sd_msgs_to_ackermann.py',
        name='sd_msgs_to_ackermann',
    )

    # Add a static_transform_publisher for fixed transforms, for example between base_link and another frame
    static_tf_publisher_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        arguments=[ PythonExpression(["'", LaunchConfiguration('robot_pose'), "'.split(',')[0]"]),
                    PythonExpression(["'", LaunchConfiguration('robot_pose'), "'.split(',')[1]"]),
                    PythonExpression(["'", LaunchConfiguration('robot_pose'), "'.split(',')[2]"]),
                    PythonExpression(["'", LaunchConfiguration('robot_pose'), "'.split(',')[3]"]),
                    PythonExpression(["'", LaunchConfiguration('robot_pose'), "'.split(',')[4]"]),
                    PythonExpression(["'", LaunchConfiguration('robot_pose'), "'.split(',')[5]"]),
                    'world', 'odom'],  # Example values for static transform
        output='screen'
    )
    
    # Add a node to publish the odometry transform from odom to base_link
    odom_tf_broadcaster_node = Node(
        package='air_sim',
        executable='odom_tf_broadcaster.py',
        name='odom_tf_broadcaster',
        output='screen'
    )
    
    return LaunchDescription([
        rviz_arg,
        declare_world_name_arg,
        declare_robot_pose_arg,
        robot_state_publisher_node,
        gazebo_server,
        gazebo_client,
        urdf_spawn_node,
        rviz2_node,
        sd_msgs_to_ackermann,
        static_tf_publisher_node,
        odom_tf_broadcaster_node,
    ])
