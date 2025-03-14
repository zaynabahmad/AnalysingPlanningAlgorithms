import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import LogInfo, ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    model_file = os.path.join(get_package_share_directory('model_description'), 'models', 'model.sdf')
    rviz_config_file = os.path.join(get_package_share_directory('model_description'),
                                    'rviz', 'model.rviz')

    return LaunchDescription([
        LogInfo(msg='Launching Ignition Gazebo with the model...'),
        
        # Start Ignition Gazebo with the specified model file
        ExecuteProcess(
            cmd=['ign', 'gazebo', '-r', model_file],
            output='screen'
        ),
        
        # Start the ROS-Ignition bridge with remapping for odom
        Node(
            package='ros_ign_bridge',
            executable='parameter_bridge',
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock@ignition.msgs.Clock',
                '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
                # Remap Ignition odometry to ROS /odom:
                '/model/turtlebot3_burger/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
                '/scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan',
                '/imu@sensor_msgs/msg/Imu@ignition.msgs.IMU',
                '/model/turtlebot3_burger/tf@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V',

            ],
            remappings=[
                 ('/model/turtlebot3_burger/tf', '/tf'),
                ('/model/turtlebot3_burger/odometry', '/odom')
         ],
            output='screen'
        ),

     Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    arguments=['0', '0', '0', '0', '0', '0', 'turtlebot3_burger/base', 'turtlebot3_burger/lidar/hls_lfcd_lds'],
    output='screen'
     ),

    # Node(
    # package='tf2_ros',
    # executable='static_transform_publisher',
    # arguments=['0', '0', '0', '0', '0', '0', '/base_link', '/scan'],
    # output='screen'
    #  ),

    # Publish a static transform from "base" to "turtlebot3_burger/base"
    Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'base', 'turtlebot3_burger/base'],
        output='screen'
    ),

    # Publish a static transform from "odom" to "turtlebot3_burger/base"
    # WARNING: Typically, odometry provides a dynamic transform. Use this only if you are sure
    Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'turtlebot3_burger/base'],
        output='screen'
    ),
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     arguments=['-d', rviz_config_file],
        #     output='screen'
        # ),
    # Node(
    #     package='gap_follower',
    #     executable='optimized_gap_follower',
    #     name='optimized_gap_follower',
    #     output='screen'
    # )

    #     Node(
    #     package='gap_follower',
    #     executable='optimized_gap_follower2',
    #     name='optimized_gap_follower2',
    #     output='screen'
    # )

          Node(
        package='gap_follower',
        executable='optimized_gapfolloweSM',
        name='optimized_gapfolloweSM',
        output='screen'
    )

    ])
