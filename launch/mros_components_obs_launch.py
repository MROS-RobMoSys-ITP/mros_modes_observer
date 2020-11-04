# Copyright (c) 2018 Intel Corporation
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

"""This is all-in-one launch script intended for use by nav2 developers."""

import os

from ament_index_python.packages import get_package_share_directory, get_package_prefix

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, EmitEvent
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, LifecycleNode
from launch_ros.events.lifecycle import ChangeState
import launch.events
import lifecycle_msgs.msg

def generate_launch_description():
    # Get the launch directory
    modes_observer_dir = get_package_share_directory('mros_modes_observer')
     
    components_file_path = os.path.join(modes_observer_dir, 'params', 'components.yaml')

    # Start as a normal node is currently not possible.
    # Path to SHM file should be passed as a ROS parameter.
    modes_observer_node = Node(
        package='mros_modes_observer',
        executable='modes_observer_node',
        parameters=[{'componentsfile': components_file_path}],
        output='screen')

    ld = LaunchDescription()

       # Add system modes manager
    ld.add_action(modes_observer_node)

   
    return ld
