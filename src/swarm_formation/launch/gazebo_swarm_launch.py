# Copyright 2026 Open Source Robotics Foundation, Inc.
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

"""Launch Gazebo Harmonic physics simulation, spawn 5 physical robots, and launch swarm nodes."""

import os
import time

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
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
    sensing_range_str = LaunchConfiguration('sensing_range').perform(context)
    sim_duration_str = LaunchConfiguration('sim_duration').perform(context)
    headless_str = LaunchConfiguration('headless').perform(context).lower()
    world_arg = LaunchConfiguration('world').perform(context)
    params_file_arg = LaunchConfiguration('params_file').perform(context)
    fault_profile_arg = LaunchConfiguration('fault_profile').perform(context)
    controller_type_str = LaunchConfiguration('controller_type').perform(context).lower()
    use_safety_filter_str = LaunchConfiguration('use_safety_filter').perform(context).lower()
    record_bag_str = LaunchConfiguration('record_bag').perform(context).lower()
    bag_path_arg = LaunchConfiguration('bag_path').perform(context)

    total_robots = int(total_robots_str)
    enable_faults = enable_faults_str in ['true', '1', 'yes']
    use_resilient = use_resilient_str in ['true', '1', 'yes']
    use_safety_filter = use_safety_filter_str in ['true', '1', 'yes']
    record_bag = record_bag_str in ['true', '1', 'yes']
    radius = float(radius_str)
    sensing_range = float(sensing_range_str)
    sim_duration = float(sim_duration_str)
    headless = headless_str in ['true', '1', 'yes']

    # Resolve params file
    default_params = os.path.join(pkg_swarm, 'config', 'swarm_params.yaml')
    if os.path.isabs(params_file_arg) and os.path.exists(params_file_arg):
        params_file = params_file_arg
    elif os.path.exists(params_file_arg):
        params_file = os.path.abspath(params_file_arg)
    else:
        params_file = default_params

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

    # Resolve fault profile
    fault_profile = ''
    if fault_profile_arg:
        base_name = os.path.basename(fault_profile_arg)
        base_cand = os.path.join(pkg_swarm, 'config', 'fault_profiles', base_name)
        if os.path.isabs(fault_profile_arg) and os.path.exists(fault_profile_arg):
            fault_profile = fault_profile_arg
        elif os.path.exists(fault_profile_arg):
            fault_profile = os.path.abspath(fault_profile_arg)
        elif os.path.exists(base_cand):
            fault_profile = base_cand
        elif os.path.exists(base_cand + '.yaml'):
            fault_profile = base_cand + '.yaml'
        else:
            cand = os.path.join(pkg_swarm, 'config', 'fault_profiles', fault_profile_arg)
            if os.path.exists(cand):
                fault_profile = cand
            elif os.path.exists(cand + '.yaml'):
                fault_profile = cand + '.yaml'
    elif enable_faults:
        cand = os.path.join(pkg_swarm, 'config', 'fault_profiles', 'robot_death.yaml')
        if os.path.exists(cand):
            fault_profile = cand

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

        # Bridge definitions for cmd_vel, pose, and odometry
        bridge_args.append(f'/model/robot_{i}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist')
        bridge_args.append(f'/model/robot_{i}/pose@geometry_msgs/msg/Pose[gz.msgs.Pose')
        bridge_args.append(
            f'/model/robot_{i}/odometry@nav_msgs/msg/Odometry[gz.msgs.OdometryWithCovariance'
        )
        bridge_remappings.append((f'/model/robot_{i}/cmd_vel', f'/robot_{i}/cmd_vel'))
        bridge_remappings.append((f'/model/robot_{i}/pose', f'/robot_{i}/pose'))
        bridge_remappings.append((f'/model/robot_{i}/odometry', f'/robot_{i}/odom'))

        # Parameters list: load YAML params first, then apply overrides
        node_params = []
        if os.path.exists(params_file):
            node_params.append(params_file)
        node_params.append({
            'robot_id': i,
            'total_robots': total_robots,
            'enable_faults': enable_faults,
            'use_resilient_method': use_resilient,
            'use_safety_filter': use_safety_filter,
            'controller_type': controller_type_str,
            'formation_radius': radius,
            'sensing_range': sensing_range,
        })

        # Relative Neighbor & Obstacle Sensor Node (M1)
        sensor_node = Node(
            package='swarm_formation',
            executable='neighbor_sensor',
            name=f'neighbor_sensor_{i}',
            output='screen',
            parameters=node_params,
        )
        actions.append(sensor_node)

        # Onboard Cooperative EKF Localization Node (M5)
        loc_params = list(node_params)
        loc_params.append({
            'initial_x': init_x,
            'initial_y': init_y,
            'initial_yaw': 0.0,
            'publish_rate': 50.0,
            'use_landmarks': True,
        })
        localization_node = Node(
            package='swarm_formation',
            executable='localization_node',
            name=f'localization_node_{i}',
            output='screen',
            parameters=loc_params,
        )
        actions.append(localization_node)

        # Gazebo Swarm Controller Node
        controller_node = Node(
            package='swarm_formation',
            executable='robot_node',
            name=f'gazebo_node_{i}',
            output='screen',
            parameters=node_params,
        )
        actions.append(controller_node)

        # Standalone Control Barrier Function (CBF-QP) Safety Filter Layer (M6)
        if use_safety_filter:
            filter_node = Node(
                package='swarm_formation',
                executable='safety_filter',
                name=f'safety_filter_{i}',
                output='screen',
                parameters=node_params,
            )
            actions.append(filter_node)

    # 3. Standalone Network Impairment Node (M2)
    default_drop = 0.15 if enable_faults else 0.0
    default_delay = 0.08 if enable_faults else 0.0
    default_jitter = 0.04 if enable_faults else 0.0

    network_node = Node(
        package='swarm_formation',
        executable='network_impairment',
        name='network_impairment',
        output='screen',
        parameters=[
            params_file,
            {
                'total_robots': total_robots,
                'default_drop_rate': default_drop,
                'default_delay_sec': default_delay,
                'default_jitter_sec': default_jitter,
                'enable_faults': enable_faults,
            },
        ],
    )
    actions.append(network_node)

    # 4. Standalone Fault Injector Node (M3)
    if fault_profile or enable_faults:
        fault_node = Node(
            package='swarm_formation',
            executable='fault_injector',
            name='fault_injector',
            output='screen',
            parameters=[{
                'fault_profile': fault_profile,
                'total_robots': total_robots,
                'update_rate': 20.0,
            }],
        )
        actions.append(fault_node)

    # 4. Consolidated Parameter Bridge Node
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

    # 5. Standalone rosbag2 Recorder Process (M7)
    if record_bag:
        bag_out = bag_path_arg if bag_path_arg else f'bags/swarm_bag_{int(time.time())}'
        topics_to_record = [
            '/clock',
            '/swarm/network_status',
            '/swarm/fault_status',
        ]
        for i in range(1, total_robots + 1):
            topics_to_record.extend([
                f'/robot_{i}/pose',
                f'/robot_{i}/odom',
                f'/robot_{i}/cmd_vel',
                f'/robot_{i}/cmd_vel_raw',
                f'/robot_{i}/estimated_pose',
                f'/robot_{i}/estimation_error',
                f'/robot_{i}/safety_intervention',
                f'/robot_{i}/neighbors',
                f'/robot_{i}/obstacles',
            ])
        bag_cmd = [
            'ros2', 'bag', 'record',
            '-o', bag_out,
            '--max-cache-size', '0',
        ] + topics_to_record
        actions.append(
            TimerAction(
                period=1.0,
                actions=[
                    ExecuteProcess(
                        cmd=bag_cmd,
                        output='screen',
                        name='rosbag_recorder',
                    )
                ],
            )
        )

    # 6. Clean Shutdown on Metrics Completion
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
            'sensing_range',
            default_value='6.0',
            description='Onboard sensor range limit in meters.',
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
        DeclareLaunchArgument(
            'params_file',
            default_value='src/swarm_formation/config/swarm_params.yaml',
            description='Path to YAML parameter file.',
        ),
        DeclareLaunchArgument(
            'fault_profile',
            default_value='',
            description='Path or preset name for fault scenario YAML profile.',
        ),
        DeclareLaunchArgument(
            'controller_type',
            default_value='displacement',
            description='Swarm controller family: displacement, distance, bearing.',
        ),
        DeclareLaunchArgument(
            'use_safety_filter',
            default_value='true',
            description='Enable standalone Control Barrier Function (CBF-QP) safety filter.',
        ),
        DeclareLaunchArgument(
            'record_bag',
            default_value='false',
            description='Enable automatic recording of swarm topics with ros2 bag.',
        ),
        DeclareLaunchArgument(
            'bag_path',
            default_value='',
            description='Output directory path for recorded rosbag2.',
        ),
    ]

    return LaunchDescription(
        declared_args + [OpaqueFunction(function=generate_gazebo_swarm_nodes)]
    )
