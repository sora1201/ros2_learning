#*********************************************************************
 # Software License Agreement (BSD License)
 #
 #  Copyright (c) 2021, Dataspeed Inc
 #  All rights reserved.
 #
 #  Redistribution and use in source and binary forms, with or without
 #  modification, are permitted provided that the following conditions
 #  are met:
 #
 #   * Redistributions of source code must retain the above copyright
 #     notice, this list of conditions and the following disclaimer.
 #   * Redistributions in binary form must reproduce the above
 #     copyright notice, this list of conditions and the following
 #     disclaimer in the documentation and/or other materials provided
 #     with the distribution.
 #   * Neither the name of Dataspeed Inc. nor the names of its
 #     contributors may be used to endorse or promote products derived
 #     from this software without specific prior written permission.
 #
 #  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 #  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 #  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 #  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 #  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 #  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 #  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 #  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 #  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 #  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 #  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 #  POSSIBILITY OF SUCH DAMAGE.
 #********************************************************************

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, EmitEvent, RegisterEventHandler, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PythonExpression, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
  urdf_share_dir = get_package_share_directory('descriptions')
  share_dir = get_package_share_directory('gazebo_learning')
  rviz_config_file = os.path.join(share_dir, 'rviz', 'ros2_learning.rviz')
  world = os.path.join(share_dir, 'worlds', 'test_room.world')

  start_rviz_cmd = Node(
    package='rviz2',
    executable='rviz2',
    arguments=['-d', rviz_config_file],
    output='screen'
  )

  exit_event_handler = RegisterEventHandler(
    event_handler=OnProcessExit(
      target_action=start_rviz_cmd,
      on_exit=EmitEvent(event=Shutdown(reason='rviz exited'))
    )
  )

  declare_gui_cmd = DeclareLaunchArgument(
    'gui',
    default_value='True',
    description='Whether to launch the Gazebo GUI or not (headless)')
  gui = LaunchConfiguration('gui')
  start_gazebo = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(
      get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')),
    launch_arguments={'world' : world, 'gui' : gui}.items()
  )

  description = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
      os.path.join(urdf_share_dir, 'launch', 'description.launch.py')
    ])
  )

  ld = LaunchDescription()

  # Add the actions
  ld.add_action(declare_gui_cmd)
  ld.add_action(start_gazebo)
  ld.add_action(start_rviz_cmd)
  ld.add_action(exit_event_handler)
  ld.add_action(description)

  return ld
