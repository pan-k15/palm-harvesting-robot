from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_description = FindPackageShare('robot_description')
    pkg_simulation  = FindPackageShare('robot_simulation')
    pkg_ros_gz_sim  = FindPackageShare('ros_gz_sim')

    controller_params = PathJoinSubstitution(
        [pkg_description, 'config', 'ros2_control.yaml']
    )

    urdf_path    = PathJoinSubstitution([pkg_description, 'urdf', 'robot.urdf.xacro'])
    default_world = PathJoinSubstitution([pkg_simulation, 'worlds', 'empty.sdf'])

    # ── Launch arguments ──────────────────────────────────────────────
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='Absolute path to the Gazebo Sim SDF world file',
    )
    x_arg   = DeclareLaunchArgument('x',   default_value='0.0',  description='Spawn X (m)')
    y_arg   = DeclareLaunchArgument('y',   default_value='0.0',  description='Spawn Y (m)')
    z_arg   = DeclareLaunchArgument('z',   default_value='0.15', description='Spawn Z (m)')
    yaw_arg = DeclareLaunchArgument('yaw', default_value='0.0',  description='Spawn yaw (rad)')
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Open RViz alongside Gazebo Sim',
    )

    # ── Gazebo Sim  (-r = start running immediately) ──────────────────
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])
        ),
        launch_arguments={
            'gz_args': [LaunchConfiguration('world'), ' -r'],
        }.items(),
    )

    # ── Robot state publisher ─────────────────────────────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(
                Command(['xacro ', urdf_path]), value_type=str
            ),
            'use_sim_time': True,
        }],
    )

    # ── Spawn robot from the /robot_description topic ─────────────────
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name',  'ugv_rover_pt',
            '-topic', 'robot_description',
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', LaunchConfiguration('z'),
            '-Y', LaunchConfiguration('yaw'),
        ],
    )

    # ── Controller spawners ───────────────────────────────────────────
    # Triggered after spawn_robot exits so controller_manager is ready.
    # gz_ros2_control starts controller_manager inside Gazebo; /cmd_vel,
    # /odom, /tf and /joint_states are published natively on ROS 2 —
    # no bridge entries needed for those topics.
    def make_spawner(name):
        return Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                name,
                '--controller-manager', '/controller_manager',
                '--param-file', controller_params,
            ],
            output='screen',
        )

    spawner_jsb   = make_spawner('joint_state_broadcaster')
    spawner_drive = make_spawner('diff_drive_controller')
    spawner_arm   = make_spawner('arm_trajectory_controller')
    spawner_blade = make_spawner('blade_velocity_controller')

    load_controllers = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=[spawner_jsb, spawner_drive, spawner_arm, spawner_blade],
        )
    )

    # ── GZ ↔ ROS 2 topic bridge ───────────────────────────────────────
    # /cmd_vel, /odom, /tf, /joint_states are omitted — gz_ros2_control
    # controllers publish/subscribe those directly on ROS 2.
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=[
            # simulation clock
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # IMU
            '/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU',
            # camera info
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/depth_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/front_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            # depth point cloud
            '/depth_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
        ],
        parameters=[{'use_sim_time': True}],
    )

    # ── LiDAR bridge → /scan_raw (frame_id fixed by scan_relay) ──────
    scan_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='scan_bridge',
        output='screen',
        arguments=['/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'],
        remappings=[('/scan', '/scan_raw')],
        parameters=[{'use_sim_time': True}],
    )

    scan_relay = Node(
        package='robot_simulation',
        executable='scan_relay',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # ── Image bridge (uses transport, separate from parameter_bridge) ──
    image_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        output='screen',
        arguments=[
            '/camera/image_raw',
            '/depth_camera/image_raw',
            '/front_camera/image_raw',
        ],
        parameters=[{'use_sim_time': True}],
    )

    # ── RViz (optional) ───────────────────────────────────────────────
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        world_arg,
        x_arg, y_arg, z_arg, yaw_arg,
        rviz_arg,
        gz_sim,
        robot_state_publisher,
        spawn_robot,
        load_controllers,
        bridge,
        scan_bridge,
        scan_relay,
        image_bridge,
        rviz,
    ])
