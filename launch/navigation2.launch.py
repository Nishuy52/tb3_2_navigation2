# Copyright 2019 Open Source Robotics Foundation, Inc.
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

import os
import yaml
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetRemap


TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']
ROS_DISTRO = os.environ.get('ROS_DISTRO')


def rewrite_nav2_params(params_path, ns, slam_active):
    with open(params_path, 'r') as f:
        params = yaml.safe_load(f)

    def ns_frame(frame):
        if frame and not frame.startswith(ns + '/'):
            return f'{ns}/{frame}'
        return frame

    top_level_frame_keys = {
        'amcl':              ['base_frame_id', 'global_frame_id', 'odom_frame_id'],
        'bt_navigator':      ['global_frame', 'robot_base_frame'],
        'behavior_server':   ['local_frame', 'global_frame', 'robot_base_frame'],
        'recoveries_server': ['global_frame', 'robot_base_frame'],
        'collision_monitor': ['base_frame_id', 'odom_frame_id'],
    }
    for node_name, keys in top_level_frame_keys.items():
        if node_name not in params:
            continue
        p = params[node_name].get('ros__parameters', {})
        for k in keys:
            if k in p:
                p[k] = ns_frame(p[k])

    for costmap_key in ['local_costmap', 'global_costmap']:
        if costmap_key not in params:
            continue
        inner = (params[costmap_key]
                 .get(costmap_key, {})
                 .get('ros__parameters', {}))
        for frame_key in ['global_frame', 'robot_base_frame']:
            if frame_key in inner:
                inner[frame_key] = ns_frame(inner[frame_key])

    # Rewrite observation source topics in all costmap layers.
    # The scan source config is nested under each plugin key (obstacle_layer,
    # voxel_layer, etc.), not directly under ros__parameters. We must walk
    # into each plugin dict to find the observation source sub-dicts.
    #
    # YAML structure:
    #   local_costmap:
    #     local_costmap:
    #       ros__parameters:
    #         obstacle_layer:        ← plugin dict
    #           observation_sources: scan
    #           scan:                ← source dict (contains topic:)
    #             topic: /scan       ← rewrite target
    for costmap_key in ['local_costmap', 'global_costmap']:
        if costmap_key not in params:
            continue
        ros_params = (params[costmap_key]
                      .get(costmap_key, {})
                      .get('ros__parameters', {}))
        # Walk every value in ros__parameters looking for plugin dicts
        # that contain an observation_sources key.
        for plugin_dict in ros_params.values():
            if not isinstance(plugin_dict, dict):
                continue
            if 'observation_sources' not in plugin_dict:
                continue
            for source_name in str(plugin_dict['observation_sources']).split():
                source_cfg = plugin_dict.get(source_name)
                if isinstance(source_cfg, dict):
                    if source_cfg.get('topic') in ('/scan', 'scan'):
                        source_cfg['topic'] = f'/{ns}/scan'

    # Rewrite odom_topic in bt_navigator (hardcoded /odom in params).
    if 'bt_navigator' in params:
        p = params['bt_navigator'].get('ros__parameters', {})
        if p.get('odom_topic') in ('/odom', 'odom'):
            p['odom_topic'] = f'/{ns}/odom'

    if slam_active:
        if 'global_costmap' in params:
            inner = (params['global_costmap']
                     .get('global_costmap', {})
                     .get('ros__parameters', {}))
            plugins = inner.get('plugins', [])
            inner['plugins'] = [p for p in plugins if p != 'static_layer']
            inner['map_topic'] = f'/{ns}/map'

    tmp = tempfile.NamedTemporaryFile(
        mode='w', suffix='.yaml', delete=False, prefix='nav2_params_rewritten_'
    )
    yaml.dump(params, tmp)
    tmp.flush()
    return tmp.name


def rewrite_slam_params(slam_params_path, ns):
    with open(slam_params_path, 'r') as f:
        params = yaml.safe_load(f)

    p = params['slam_toolbox']['ros__parameters']
    p['odom_frame'] = f'{ns}/odom'
    p['map_frame']  = f'{ns}/map'
    p['base_frame'] = f'{ns}/base_footprint'
    p['scan_topic'] = f'/{ns}/scan'
    p['odom_topic'] = f'/{ns}/odom'
    p['map_topic']  = f'/{ns}/map'

    tmp = tempfile.NamedTemporaryFile(
        mode='w', suffix='.yaml', delete=False, prefix='slam_params_rewritten_'
    )
    yaml.dump(params, tmp)
    tmp.flush()
    return tmp.name


def launch_setup(context, *args, **kwargs):
    ns       = context.launch_configurations['namespace']
    slam_val = context.launch_configurations['slam'].lower() == 'true'
    use_sim  = context.launch_configurations['use_sim_time']
    map_yaml = context.launch_configurations['map']
    params_f = context.launch_configurations['params_file']
    slam_pf  = context.launch_configurations['slam_params_file']

    nav2_launch_file_dir = os.path.join(
        get_package_share_directory('nav2_bringup'), 'launch'
    )
    rviz_config_dir = os.path.join(
        get_package_share_directory('tb3_1_navigation2'),
        'rviz', 'tb3_navigation2.rviz'
    )

    rewritten_nav2_params = rewrite_nav2_params(params_f, ns, slam_val)

    # ------------------------------------------------------------------
    # Nav2 bringup — wrapped in GroupAction with SetRemap
    #
    # tf2_ros::Buffer inside composable nodes subscribes to /tf and
    # /tf_static at the DDS layer, bypassing node-level remappings=.
    # SetRemap inside a GroupAction is the only mechanism that propagates
    # TF remappings into composable nodes loaded inside a container,
    # because it operates at the launch graph level before any node
    # initializes its subscriptions.
    #
    # /tf        -> /{ns}/tf        so Nav2 reads slam_toolbox map->odom
    #                               and Pi odom->base_footprint from the
    #                               namespaced topic (no cross-robot bleed)
    # /tf_static -> /{ns}/tf_static so Nav2 reads robot_state_publisher's
    #                               static transforms (base_link, base_scan)
    # ------------------------------------------------------------------
    nav2_group = GroupAction([
        SetRemap('/tf',        f'/{ns}/tf'),
        SetRemap('/tf_static', f'/{ns}/tf_static'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [nav2_launch_file_dir, '/bringup_launch.py']
            ),
            launch_arguments={
                'map':           map_yaml,
                'use_sim_time':  use_sim,
                'params_file':   rewritten_nav2_params,
                'slam':          'False',
                'namespace':     ns,
                'use_namespace': 'True',
            }.items(),
        ),
    ])

    # ------------------------------------------------------------------
    # slam_toolbox — no Node namespace (avoids double-prefixing).
    # All topics wired via explicit remappings:
    #   /scan -> /{ns}/scan   (Pi lidar input)
    #   /odom -> /{ns}/odom   (Pi odometry input)
    #   /tf   -> /{ns}/tf     (map->odom output)
    #   /map  -> /{ns}/map    (occupancy grid output)
    #
    # NOTE: In Humble, slam_toolbox ignores scan_topic/odom_topic params
    # for actual subscriptions. Node remappings are required.
    # ------------------------------------------------------------------
    if slam_val:
        rewritten_slam_params = rewrite_slam_params(slam_pf, ns)

        slam_node = Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name=f'slam_toolbox_{ns}',
            parameters=[rewritten_slam_params],
            remappings=[
                ('/scan', f'/{ns}/scan'),
                ('/odom', f'/{ns}/odom'),
                ('/tf',   f'/{ns}/tf'),
                ('/map',  f'/{ns}/map'),
            ],
            output='screen',
        )

    # ------------------------------------------------------------------
    # RViz2
    # ------------------------------------------------------------------
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        namespace=ns,
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': use_sim == 'true'}],
        remappings=[
            ('/tf',        f'/{ns}/tf'),
            ('/tf_static', f'/{ns}/tf_static'),
        ],
        output='screen',
    )

    # tf_static_relay: fixes cross-machine TRANSIENT_LOCAL delivery failure.
    # The Pi publishes /{ns}/tf_static with TRANSIENT_LOCAL, but Fast DDS
    # does not reliably deliver cached messages to late-joining subscribers
    # across machine boundaries. This relay subscribes VOLATILE (catching
    # whatever arrives) and republishes locally with TRANSIENT_LOCAL so all
    # local Nav2 nodes receive the full static TF tree regardless of startup order.
    tf_static_relay = Node(
        package='tb3_1_navigation2',
        executable='tf_static_relay.py',
        name='tf_static_relay',
        output='screen',
        arguments=[f'/{ns}/tf_static'],
    )

    actions = [tf_static_relay, nav2_group]
    if slam_val:
        actions.append(slam_node)
    actions.append(rviz_node)
    return actions


def generate_launch_description():
    param_file_name = TURTLEBOT3_MODEL + '.yaml'

    if ROS_DISTRO == 'humble':
        default_params = os.path.join(
            get_package_share_directory('tb3_1_navigation2'),
            'param', ROS_DISTRO, param_file_name
        )
    else:
        default_params = os.path.join(
            get_package_share_directory('tb3_1_navigation2'),
            'param', param_file_name
        )

    default_map = os.path.join(
        get_package_share_directory('tb3_1_navigation2'),
        'map', 'map.yaml'
    )
    default_slam_params = os.path.join(
        get_package_share_directory('tb3_1_navigation2'),
        'param', 'slam_toolbox_params.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='tb3_1',
            description='Namespace for the robot'
        ),
        DeclareLaunchArgument(
            'map',
            default_value=default_map,
            description='Full path to map yaml file to load'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Full path to the nav2 param file'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'slam',
            default_value='True',
            description='Use slam_toolbox for live mapping (True) or static map (False)'
        ),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=default_slam_params,
            description='Full path to the slam_toolbox param file'
        ),

        OpaqueFunction(function=launch_setup),
    ])