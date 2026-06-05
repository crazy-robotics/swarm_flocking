"""
Launch file for swarm_flocking: spawns N agents targeting PX4 SITL instances.

Usage:
  ros2 launch swarm_flocking swarm.launch.py num_agents:=3 v_flock:=4.0

PX4 multi-vehicle DDS topic naming:
  Instance 0 (agent 1):  /fmu/out/vehicle_odometry, /fmu/in/trajectory_setpoint, ...
  Instance 1 (agent 2):  /px4_1/fmu/out/vehicle_odometry, /px4_1/fmu/in/trajectory_setpoint, ...
  Instance N (agent N+1): /px4_N/fmu/out/vehicle_odometry, /px4_N/fmu/in/trajectory_setpoint, ...

The SwarmController node computes the correct topic names internally from
agent_id, so no remappings are required for PX4 topics.

Each agent runs as an independent node with its own namespace, communicating
with its peer agents via the shared /swarm/ topics (DDS multicast).
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_agents(context):
    num_agents      = int(LaunchConfiguration('num_agents').perform(context))
    v_flock         = float(LaunchConfiguration('v_flock').perform(context))
    arena_size      = float(LaunchConfiguration('arena_size').perform(context))
    flight_altitude = float(LaunchConfiguration('flight_altitude').perform(context))
    auto_takeoff    = LaunchConfiguration('auto_takeoff').perform(context).lower() == 'true'
    config_file     = LaunchConfiguration('config_file').perform(context)

    pkg_dir = get_package_share_directory('swarm_flocking')
    default_config = os.path.join(pkg_dir, 'config', 'flocking_params.yaml')
    config = config_file if config_file else default_config

    nodes = []
    for i in range(1, num_agents + 1):
        # PX4 topic prefix computed in SwarmController::px4_topic_prefix():
        #   agent 1 → ""         → /fmu/out/vehicle_odometry
        #   agent 2 → "/px4_1"  → /px4_1/fmu/out/vehicle_odometry
        #   agent 3 → "/px4_2"  → /px4_2/fmu/out/vehicle_odometry
        # No remappings needed for PX4 topics — the node handles prefix internally.
        node = Node(
            package='swarm_flocking',
            executable='swarm_node',
            name=f'swarm_agent_{i}',
            namespace=f'agent_{i}',
            output='screen',
            parameters=[
                config,
                {
                    'agent_id': i,
                    'flock.v_flock': v_flock,
                    'flock.arena_size': arena_size,
                    'flight_altitude_m': flight_altitude,
                    'auto_takeoff': auto_takeoff,
                }
            ],
            # No remappings: PX4 topics are constructed inside the node
            # from agent_id. Swarm topics (/swarm/*) are global and shared.
        )
        nodes.append(node)

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('num_agents', default_value='3',
                              description='Number of drones (must match running PX4 SITL instances)'),
        DeclareLaunchArgument('v_flock', default_value='4.0',
                              description='Desired flocking speed (m/s)'),
        DeclareLaunchArgument('arena_size', default_value='200.0',
                              description='Square arena side length (m)'),
        DeclareLaunchArgument('flight_altitude', default_value='5.0',
                              description='Target flight altitude in meters AGL (all agents converge here)'),
        DeclareLaunchArgument('auto_takeoff', default_value='true',
                              description='true = auto arm+takeoff+offboard on startup; false = arm manually via QGC'),
        DeclareLaunchArgument('config_file', default_value='',
                              description='Path to custom config YAML (optional)'),
        OpaqueFunction(function=generate_agents),
    ])
