# Copyright 2022 ICube Laboratory, University of Strasbourg
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    
    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare('haptic_description'), 'config', 'haptic.config.xacro']
            ),
        ]
    )
    robot_description = {'robot_description': robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare('haptic_description'),
            'config',
            'haptic_controllers.yaml',
        ]
    )
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare('haptic_description'), 'rviz', 'haptic.rviz']
    )

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, robot_controllers],
        output='both',
    )
    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description],
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )

    trajectory_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['haptic_trajectory_controller'],
        )
    
    effort_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['forward_effort_controller'],
        )

    pseudo_trajectory = Node(
        package='haptic_nodes',
        executable='pseudo_trajectory.py',
        name='pseudo_trajectory',
    )

    haptic_loop_node = Node(
        package='haptic_nodes',
        executable='haptic_loop.py',
        name='haptic_loop_node',
    )

    markers_node = Node(
        package='haptic_nodes',
        executable='markers.py',
        name='markers_node',
    )

    nodes = [
        rviz_node,
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        pseudo_trajectory,
        haptic_loop_node,
        markers_node,
        trajectory_controller_spawner,
        #effort_controller_spawner,
    ]

    return LaunchDescription(nodes)