import os
import launch
import launch_ros

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import FindExecutable, PathJoinSubstitution
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    model_name = 'dtw_robot'
    xacro = model_name + '.urdf.xacro'
    urdf_share_dir = get_package_share_directory('descriptions')
    xacro_path = os.path.join(urdf_share_dir, 'urdf', model_name, xacro)

    declare_gpu_cmd = DeclareLaunchArgument(
    'gpu',
    default_value='False',
    description='Whether to use Gazebo gpu_ray or ray')
    declare_organize_cloud_cmd = DeclareLaunchArgument(
        'organize_cloud',
        default_value='False',
        description='Organize PointCloud2 into 2D array with NaN placeholders, otherwise 1D array and leave out invlaid points')
    gpu = LaunchConfiguration('gpu')
    organize_cloud = LaunchConfiguration('organize_cloud')
    # robot_description = Command(['xacro',' ', xacro_path, ' gpu:=', gpu, ' organize_cloud:=', organize_cloud])
    robot_description = Command(['xacro',' ', xacro_path])

    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': robot_description
        }]
    )

    spawn_example_cmd = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'dtw_robot',
            '-topic', 'robot_description'
        ],
        output='screen',
    )

    joint_state_publisher_cmd = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen"
    ),
    
    ld = LaunchDescription()

    # ld.add_action(declare_gpu_cmd)
    # ld.add_action(declare_organize_cloud_cmd)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(spawn_example_cmd)
    # ld.add_action(joint_state_publisher_cmd)

    return ld
