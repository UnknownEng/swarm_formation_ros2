# Copyright 2026 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the 'License');
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an 'AS IS' BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Launch Gazebo Harmonic physics simulation, spawn 5 physical robots, and launch swarm nodes."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_gazebo_swarm_nodes(context: LaunchContext, *args, **kwargs):
    """Generate launch actions for Gazebo Harmonic, model spawners, bridges, and nodes."""
    pkg_swarm = get_package_share_directory('swarm_formation')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    urdf_path = os.path.join(pkg_swarm, 'urdf', 'robot.urdf')

    total_robots_str = LaunchConfiguration('total_robots').perform(context)
    enable_faults_str = LaunchConfiguration('enable_faults').perform(context).lower()
    use_resilient_str = LaunchConfiguration('use_resilient_method').perform(context).lower()
    csv_path = LaunchConfiguration('csv_path').perform(context)
    radius_str = LaunchConfiguration('formation_radius').perform(context)
    sim_duration_str = LaunchConfiguration('sim_duration').perform(context)
    headless_str = LaunchConfiguration('headless').perform(context).lower()
    world_arg = LaunchConfiguration('world').perform(context)

    total_robots = int(total_robots_str)
    enable_faults = enable_faults_str in ['true', '1', 'yes']
    use_resilient = use_resilient_str in ['true', '1', 'yes']
    radius = float(radius_str)
    sim_duration = float(sim_duration_str)
    headless = headless_str in ['true', '1', 'yes']

    # Resolve world path
    if os.path.isabs(world_arg) and os.path.exists(world_arg):
        world_path = world_arg
    elif os.path.exists(world_arg):
        world_path = os.path.abspath(world_arg)
    else:
        pkg_world = os.path.join(pkg_swarm, 'worlds', os.path.basename(world_arg))
        if os.path.exists(pkg_world):
            world_path = pkg_world
        else:
            default_world = os.path.join(pkg_swarm, 'worlds', 'collapsed_building.sdf')
            world_path = default_world if os.path.exists(default_world) else 'empty.sdf'

    world_name = 'collapsed_building' if 'collapsed_building' in world_path else 'empty'
    gz_args = f'-r -s {world_path}' if headless else f'-r {world_path}'

    actions = []

    # 1. Gazebo Harmonic Simulation Process
    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': gz_args}.items(),
    )
    actions.append(gz_sim_launch)

    # Crossing trajectory initial positions to verify collisions vs APF safety
    preset_positions = [
        (-3.0, 0.0),
        (3.0, 0.0),
        (0.0, -3.0),
        (0.0, 3.0),
        (2.2, -2.2),
    ]

    bridge_args = ['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock']
    bridge_remappings = []

    # 2. Spawn each robot and prepare bridge arguments
    for i in range(1, total_robots + 1):
        if i <= len(preset_positions):
            init_x, init_y = preset_positions[i - 1]
        else:
            init_x = float(i) * 1.5 - 4.0
            init_y = 0.0

        # Spawner node
        spawner = Node(
            package='ros_gz_sim',
            executable='create',
            name=f'spawner_robot_{i}',
            output='screen',
            arguments=[
                '-world', world_name,
                '-file', urdf_path,
                '-name', f'robot_{i}',
                '-x', str(init_x),
                '-y', str(init_y),
                '-z', '0.05',
            ],
        )
        actions.append(spawner)

        # Bridge definitions for cmd_vel and pose
        bridge_args.append(f'/model/robot_{i}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist')
        bridge_args.append(f'/model/robot_{i}/pose@geometry_msgs/msg/Pose[gz.msgs.Pose')
        bridge_remappings.append((f'/model/robot_{i}/cmd_vel', f'/robot_{i}/cmd_vel'))
        bridge_remappings.append((f'/model/robot_{i}/pose', f'/robot_{i}/pose'))

        # Gazebo Swarm Controller Node
        controller_node = Node(
            package='swarm_formation',
            executable='gazebo_node',
            name=f'gazebo_node_{i}',
            output='screen',
            parameters=[{
                'robot_id': i,
                'total_robots': total_robots,
                'enable_faults': enable_faults,
                'use_resilient_method': use_resilient,
                'formation_radius': radius,
                'control_rate': 50.0,
                'max_linear_vel': 1.4,
                'max_angular_vel': 3.0,
            }],
        )
        actions.append(controller_node)

    # 3. Consolidated Parameter Bridge Node
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_ros_bridge',
        output='screen',
        arguments=bridge_args,
        remappings=bridge_remappings,
    )
    actions.append(TimerAction(period=2.5, actions=[bridge_node]))

    # 4. Metrics Logger Node
    metrics_node = Node(
        package='swarm_formation',
        executable='metrics_logger',
        name='swarm_metrics_logger',
        output='screen',
        parameters=[{
            'total_robots': total_robots,
            'formation_radius': radius,
            'convergence_threshold': 0.25,
            'settle_duration': 2.0,
            'csv_path': csv_path,
            'auto_shutdown_sec': sim_duration,
            'enable_faults': enable_faults,
            'use_resilient_method': use_resilient,
        }],
    )
    actions.append(TimerAction(period=3.5, actions=[metrics_node]))

    # 5. Clean Shutdown on Metrics Completion
    if sim_duration > 0.0:
        actions.append(
            RegisterEventHandler(
                OnProcessExit(
                    target_action=metrics_node,
                    on_exit=[EmitEvent(event=Shutdown())],
                )
            )
        )

    return actions


def generate_launch_description() -> LaunchDescription:
    """Declare launch arguments and return LaunchDescription."""
    declared_args = [
        DeclareLaunchArgument(
            'total_robots',
            default_value='5',
            description='Total number of physical robots to spawn in Gazebo.',
        ),
        DeclareLaunchArgument(
            'enable_faults',
            default_value='false',
            description='Enable packet loss (15%), delay, noise, and agent 4 dropout.',
        ),
        DeclareLaunchArgument(
            'use_resilient_method',
            default_value='false',
            description='Enable resilient consensus and APF safety barrier.',
        ),
        DeclareLaunchArgument(
            'formation_radius',
            default_value='2.0',
            description='Target polygon radius in meters.',
        ),
        DeclareLaunchArgument(
            'csv_path',
            default_value='swarm_metrics.csv',
            description='CSV output destination for quantitative metrics.',
        ),
        DeclareLaunchArgument(
            'sim_duration',
            default_value='0.0',
            description='Auto-shutdown duration in seconds (0.0 = manual Ctrl-C).',
        ),
        DeclareLaunchArgument(
            'headless',
            default_value='true',
            description='Run Gazebo in headless server mode (true/false).',
        ),
        DeclareLaunchArgument(
            'world',
            default_value='src/swarm_formation/worlds/collapsed_building.sdf',
            description='Path or filename of the Gazebo SDF world file.',
        ),
    ]

    return LaunchDescription(
        declared_args + [OpaqueFunction(function=generate_gazebo_swarm_nodes)]
    )
