# file
#
#  \author Emmanuel Dean
#
#  \version 0.1
#  \date 15.03.2021
#
#  \copyright Copyright 2021 Chalmers
#
# License
#   Licensed under the Apache License, Version 2.0 (the "License");
#   you may not use this file except in compliance with the License.
#   You may obtain a copy of the License at
#
#    http://www.apache.org/licenses/LICENSE-2.0
#
#   Unless required by applicable law or agreed to in writing, software
#   distributed under the License is distributed on an "AS IS" BASIS,
#   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#   See the License for the specific language governing permissions and
#   limitations under the License.
#


import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('turtle_ctrl')

    # Launch configuration variables specific to simulation
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_rviz = LaunchConfiguration('use_rviz')

    # Define arguments
    declare_rviz_config_file_cmd = DeclareLaunchArgument('rviz_config_file', default_value=os.path.join(
        bringup_dir, 'rviz', 'turtle.rviz'), description='Full path to the RVIZ config file to use')
    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Whether to start RVIZ')

    # RQT
    rqt_graph_node = Node(package='rqt_graph',
                          executable='rqt_graph', output='screen')

    # Rviz node
    rviz_cmd = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen')

    #-------- NODES

    # Turtle Visualization
    turtle_vis_yaml = os.path.join(
        get_package_share_directory('turtle_ctrl'),
        'config',
        'turtle_vis.yaml'
    )
    turtle_vis = Node(package='turtle_ctrl',
                      name='turtle_visualizer',
                      executable='turtle_vis',
                      parameters=[turtle_vis_yaml],
                      output='screen')

    # Turtle Control
    turtle_ctrl_yaml = os.path.join(
        get_package_share_directory('turtle_ctrl'),
        'config',
        'turtle_ctrl.yaml'
    )
    turtle_ctrl = Node(package='turtle_ctrl',
                       name='turtle_control',
                       executable='turtle_control',
                       parameters=[turtle_ctrl_yaml],
                       output='screen')

    # Twist2CMD
    twist2cmd = Node(package='turtle_trajectory_generator',
                     name='twist2cmd',
                     executable='twist2cmd',
                     output='screen')

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_rviz_cmd)

    # Load nodes (actions)
    ld.add_action(rqt_graph_node)
    ld.add_action(rviz_cmd)
    ld.add_action(turtle_vis)
    ld.add_action(turtle_ctrl)
    # ld.add_action(twist2cmd)

    return ld
