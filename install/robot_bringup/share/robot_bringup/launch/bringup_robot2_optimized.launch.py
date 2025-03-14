import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import LogInfo, ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    # Get the absolute path of the model file
    model_file = os.path.join(get_package_share_directory('model_description'), 'models', 'model2.sdf')

    # Ensure the file is properly referenced using 'file://'
    model_file_uri = f'{model_file}'

    rviz_config_file = os.path.join(get_package_share_directory('model_description'),
                                    'rviz', 'model.rviz')

    return LaunchDescription([
        LogInfo(msg='Launching Ignition Gazebo with the model...'),
        
        # Start Ignition Gazebo with the specified model file
        ExecuteProcess(
            cmd=['ign', 'gazebo', '-r', model_file_uri],
            output='screen'
        ),
        
        # Start the ROS-Ignition bridge with remapping for odometry
        Node(
            package='ros_ign_bridge',
            executable='parameter_bridge',
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock@ignition.msgs.Clock',
                '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
                # Updated to match `my_robot` model
                '/model/my_robot/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
                '/scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan',
                '/imu@sensor_msgs/msg/Imu@ignition.msgs.IMU',
                '/model/my_robot/tf@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V',
            ],
            remappings=[
                ('/model/my_robot/tf', '/tf'),
                ('/model/my_robot/odometry', '/odom')
            ],
            output='screen'
        ),

        # Static transform publisher: base to lidar
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'my_robot/base', 'my_robot/lidar'],
            output='screen'
        ),

        # Static transform publisher: base to world
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'my_robot/base'],
            output='screen'
        ),

        # Static transform publisher: odom to base
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'my_robot/base'],
            output='screen'
        ),

        # Optional: RViz visualization
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     arguments=['-d', rviz_config_file],
        #     output='screen'
        # ),

        # Launch your custom node (adjust package and executable names if needed)
        Node(
            package='gap_follower',
            executable='optimized_gapfolloweSM',
            name='optimized_gapfolloweSM',
            output='screen'
        )
    ])
