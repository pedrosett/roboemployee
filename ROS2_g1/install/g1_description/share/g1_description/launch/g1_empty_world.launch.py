import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare arguments
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='empty.world',
        description='World file name'
    )
    
    # Get package directory
    pkg_share = FindPackageShare('g1_description')
    
    # Gazebo launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={
            'gz_args': '-r empty.sdf'
        }.items()
    )
    
    # Robot state publisher  
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': open(
                os.path.join(
                    os.path.dirname(__file__), '..', 
                    'g1_29dof_rev_1_0_with_inspire_hand_DFQ.urdf'
                )
            ).read()
        }],
        output='screen'
    )
    
    # Joint state publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen'
    )
    
    # Spawn robot in Gazebo
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', os.path.join(
                os.path.dirname(__file__), '..', 
                'g1_29dof_rev_1_0_with_inspire_hand_DFQ.urdf'
            ),
            '-name', 'unitree_g1',
            '-x', '0',
            '-y', '0', 
            '-z', '1.0'
        ],
        output='screen'
    )
    
    return LaunchDescription([
        world_arg,
        gazebo_launch,
        robot_state_publisher,
        joint_state_publisher,
        spawn_robot
    ])