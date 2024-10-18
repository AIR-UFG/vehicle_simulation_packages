from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
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
    
    rviz_file = os.path.join(rviz_dir, 'config', 'odom.rviz')
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

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_file],
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('rviz'), "'=='true'"]))
    )
        
    #Nodes to launch
    velodyne_driver_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('velodyne_driver'), 'launch'),
            '/velodyne_driver_node-VLP16-launch.py']),
    )
    velodyne_transform_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('velodyne_pointcloud'), 'launch'),
            '/velodyne_transform_node-VLP16-launch.py']),
    )

    rtabmap_odom = Node(
        package='rtabmap_odom', executable='icp_odometry', output='screen',
        parameters=[{
            'frame_id':'velodyne',
            'odom_frame_id':'odom',
            'wait_for_transform':0.2,
            'expected_update_rate':15.0,
            'deskewing':True,
            # RTAB-Map's internal parameters are strings:
            'Icp/PointToPlane': 'true',
            'Icp/Iterations': '10',
            'Icp/VoxelSize': '0.1',
            'Icp/Epsilon': '0.001',
            'Icp/PointToPlaneK': '20',
            'Icp/PointToPlaneRadius': '0',
            'Icp/MaxTranslation': '2',
            'Icp/MaxCorrespondenceDistance': '1',
            'Icp/Strategy': '1',
            'Icp/OutlierRatio': '0.7',
            'Icp/CorrespondenceRatio': '0.01',
            'Odom/ScanKeyFrameThr': '0.4',
            'OdomF2M/ScanSubtractRadius': '0.1',
            'OdomF2M/ScanMaxSize': '15000',
            'OdomF2M/BundleAdjustment': 'false'
        }],
        remappings=[
            ('scan_cloud', '/velodyne_points')
        ])
    
    # rtabmap_slam = Node(
    #     package='rtabmap_slam', executable='rtabmap', output='screen',
    #     parameters=[{
    #         'frame_id':'velodyne',
    #         'subscribe_depth':False,
    #         'subscribe_rgb':False,
    #         'subscribe_scan_cloud':True,
    #         'approx_sync':False,
    #         'wait_for_transform':0.2,
    #         'use_sim_time':False,
    #         # RTAB-Map's internal parameters are strings:
    #         'RGBD/ProximityMaxGraphDepth': '0',
    #         'RGBD/ProximityPathMaxNeighbors': '1',
    #         'RGBD/AngularUpdate': '0.05',
    #         'RGBD/LinearUpdate': '0.05',
    #         'RGBD/CreateOccupancyGrid': 'false',
    #         'Mem/NotLinkedNodesKept': 'false',
    #         'Mem/STMSize': '30',
    #         'Mem/LaserScanNormalK': '20',
    #         'Reg/Strategy': '1',
    #         'Icp/VoxelSize': '0.1',
    #         'Icp/PointToPlaneK': '20',
    #         'Icp/PointToPlaneRadius': '0',
    #         'Icp/PointToPlane': 'true',
    #         'Icp/Iterations': '10',
    #         'Icp/Epsilon': '0.001',
    #         'Icp/MaxTranslation': '3',
    #         'Icp/MaxCorrespondenceDistance': '1',
    #         'Icp/Strategy': '1',
    #         'Icp/OutlierRatio': '0.7',
    #         'Icp/CorrespondenceRatio': '0.2'
    #     }],
    #     remappings=[
    #         ('scan_cloud', 'odom_filtered_input_scan')
    #     ],
    #     arguments=[
    #         '-d' # This will delete the previous database (~/.ros/rtabmap.db)
    #     ]), 
        
    # rtabmap_viz = Node(
    #     package='rtabmap_viz', executable='rtabmap_viz', output='screen',
    #     parameters=[{
    #         'frame_id':'velodyne',
    #         'odom_frame_id':'odom',
    #         'subscribe_odom_info':True,
    #         'subscribe_scan_cloud':True,
    #         'approx_sync':False,
    #     }],
    #     remappings=[
    #         ('scan_cloud', 'odom_filtered_input_scan')
    #     ]),

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_gui': 'false'}],
    )

    return LaunchDescription([
        rviz_arg,
        robot_state_publisher_node,
        rviz2_node,
        velodyne_driver_node,
        velodyne_transform_node,
        rtabmap_odom,
        joint_state_publisher_node,
        # rtabmap_slam,
        # rtabmap_viz,
    ])
