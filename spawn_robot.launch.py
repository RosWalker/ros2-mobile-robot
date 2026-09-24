import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import xacro

def generate_launch_description():

    pkg_path = get_package_share_directory('rescue_robot')
    urdf_file = os.path.join(pkg_path, 'urdf', 'robot.urdf.xacro')
    world_file = os.path.join(pkg_path, 'worlds', 'assignment_world.sdf')

    # Process the xacro file into URDF
    robot_description = xacro.process_file(urdf_file).toxml()

    # ── Robot state publisher ────────────────────────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # ── Start Gazebo with the world ──────────────────────────────
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', world_file, '-r'],
        output='screen'
    )

    # ── Spawn the robot at x=0 y=0 ──────────────────────────────
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'rescue_robot',
            '-topic', 'robot_description',
            '-x', '0',
            '-y', '0',
            '-z', '0.1',
            '-R', '0',
            '-P', '0',
            '-Y', '0'
        ],
        output='screen'
    )

    # ── ROS GZ Bridge ────────────────────────────────────────────
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
            '/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
        ],
        output='screen'
    )

    # ── Navigation node ──────────────────────────────────────────
    navigation_node = Node(
        package='rescue_robot',
        executable='navigation_node.py',
        name='navigation_node',
        output='screen',
    )

    # ── Perception node ──────────────────────────────────────────
    perception_node = Node(
        package='rescue_robot',
        executable='perception_node.py',
        name='perception_node',
        output='screen',
    )

    # ── Battery node ─────────────────────────────────────────────
    battery_node = Node(
        package='rescue_robot',
        executable='battery_node.py',
        name='battery_node',
        output='screen',
    )

    # ── Task manager node ────────────────────────────────────────
    task_manager_node = Node(
        package='rescue_robot',
        executable='task_manager_node.py',
        name='task_manager_node',
        output='screen',
    )

    return LaunchDescription([
        robot_state_publisher,
        gazebo,
        spawn_robot,
        bridge,
        navigation_node,
        perception_node,
        battery_node,
        task_manager_node,
    ])
