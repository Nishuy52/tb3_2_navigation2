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
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']
ROS_DISTRO = os.environ.get('ROS_DISTRO')


def rewrite_nav2_params(params_path, ns, slam_active):
    """
    Rewrites nav2 params at launch time:
      - Prefixes all frame IDs with the robot namespace
      - When SLAM is active:
          * Removes static_layer from global costmap plugins
          * Points global costmap map_topic at slam_toolbox's live map
      - amcl and map_server are left intact so Humble's lifecycle manager
        starts cleanly. amcl's map→odom will be overridden by slam_toolbox's
        transform which arrives later and takes precedence in the TF tree.
    """
    with open(params_path, 'r') as f:
        params = yaml.safe_load(f)

    def ns_frame(frame):
        if frame and not frame.startswith(ns + '/'):
            return f'{ns}/{frame}'
        return frame

    # --- Namespace all frame IDs ---
    top_level_frame_keys = {
        'amcl':             ['base_frame_id', 'global_frame_id', 'odom_frame_id'],
        'bt_navigator':     ['global_frame', 'robot_base_frame'],
        'behavior_server':  ['local_frame', 'global_frame', 'robot_base_frame'],
        'recoveries_server':['global_frame', 'robot_base_frame'],
        'collision_monitor':['base_frame_id', 'odom_frame_id'],
    }
    for node_name, keys in top_level_frame_keys.items():
        if node_name not in params:
            continue
        p = params[node_name].get('ros__parameters', {})
        for k in keys:
            if k in p:
                p[k] = ns_frame(p[k])

    # Double-nested costmaps (Humble style)
    for costmap_key in ['local_costmap', 'global_costmap']:
        if costmap_key not in params:
            continue
        inner = (params[costmap_key]
                 .get(costmap_key, {})
                 .get('ros__parameters', {}))
        for frame_key in ['global_frame', 'robot_base_frame']:
            if frame_key in inner:
                inner[frame_key] = ns_frame(inner[frame_key])

    if slam_active:
        # Global costmap: drop static_layer so the old static map file is
        # never displayed. The costmap uses slam_toolbox's live occupancy
        # grid (relayed to /ns/map) and updates in real time.
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
    """
    Injects namespaced frame IDs and fully absolute topic paths into the
    slam_toolbox params. slam_toolbox runs WITHOUT a ROS namespace on the
    Node to avoid double-prefixing (e.g. /tb3_1/tb3_1/scan), so all
    topic paths must be absolute /ns/topic strings.
    """
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

    pkg_dir = get_package_share_directory('tb3_1_navigation2')

    nav2_launch_file_dir = os.path.join(
        get_package_share_directory('nav2_bringup'), 'launch'
    )
    rviz_config_dir = os.path.join(
        pkg_dir, 'rviz', 'tb3_navigation2.rviz'
    )

    rewritten_nav2_params = rewrite_nav2_params(params_f, ns, slam_val)

    # ------------------------------------------------------------------
    # Relay nodes
    #
    # /tf:
    #   The robot hardware stack publishes dynamic transforms to root /tf
    #   with VOLATILE / BEST_EFFORT QoS. Nav2 (namespaced) listens on
    #   /{ns}/tf. tf_relay.py bridges them with proper QoS matching — it
    #   subscribes VOLATILE/BEST_EFFORT (matching the Pi bringup publisher)
    #   and republishes VOLATILE/RELIABLE (matching Nav2's expectation).
    #
    #   NOTE: topic_tools relay was previously used here but is replaced by
    #   tf_relay.py because topic_tools does not handle QoS profiles correctly,
    #   leading to missed messages during the Nav2 startup window.
    #
    # /tf_static:
    #   Same issue but worse — static TFs are TRANSIENT_LOCAL (latched).
    #   topic_tools relay uses VOLATILE, causing Nav2 nodes that start after
    #   the initial publish to never receive static TFs. tf_static_relay.py
    #   accumulates all static transforms and republishes the complete set
    #   with TRANSIENT_LOCAL so late-joining subscribers always get them.
    #
    # /map → /{ns}/map:
    #   slam_toolbox publishes the live occupancy grid to root /map.
    #   Nav2's global costmap subscribes to /{ns}/map. map_relay.py bridges
    #   them with TRANSIENT_LOCAL depth=1 so the costmap never misses the
    #   map even if it subscribes after the initial publish.
    # ------------------------------------------------------------------
    tf_relay = Node(
        package='tb3_1_navigation2',
        executable='tf_relay.py',
        name='tf_relay',
        output='screen',
        arguments=[f'/{ns}/tf'],
    )

    tf_static_relay = Node(
        package='tb3_1_navigation2',
        executable='tf_static_relay.py',
        name='tf_static_relay',
        output='screen',
        arguments=[f'/{ns}/tf_static'],
    )

    map_relay = Node(
        package='tb3_1_navigation2',
        executable='map_relay.py',
        name='map_relay',
        output='screen',
        arguments=[f'/{ns}/map'],
    )

    # ------------------------------------------------------------------
    # Nav2 bringup
    #
    # slam:=False — we launch slam_toolbox ourselves below.
    # map:=map_yaml always — Humble's map_server requires a valid
    # yaml_filename to configure successfully. The static map it loads is
    # irrelevant during SLAM because static_layer is removed from the
    # global costmap params above.
    # ------------------------------------------------------------------
    nav2_include = IncludeLaunchDescription(
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
    )

    actions = [tf_relay, tf_static_relay, map_relay, nav2_include]

    # ------------------------------------------------------------------
    # slam_toolbox — intentionally NOT namespaced on the Node.
    #
    # When namespace='tb3_1' is set on the Node, ROS 2 prepends it to all
    # relative topic subscriptions. Combined with the absolute paths in
    # params (scan_topic: /tb3_1/scan), this resolves to /tb3_1/tb3_1/scan
    # — a topic that doesn't exist — so slam_toolbox receives no scans.
    #
    # Without a namespace, absolute paths resolve correctly:
    #   scan_topic:  /tb3_1/scan  → /tb3_1/scan  ✓
    #   odom_topic:  /tb3_1/odom  → /tb3_1/odom  ✓
    #   map_frame:   tb3_1/map                    ✓
    #
    # node_name is set to slam_toolbox_{ns} so that when multiple robots
    # are eventually run simultaneously, their slam_toolbox service
    # interfaces are distinguishable:
    #   /slam_toolbox_tb3_1/save_map  (vs /slam_toolbox_tb3_1/save_map etc.)
    #
    # slam_toolbox publishes to root topics, bridged by the relays:
    #   /map  → map_relay      → /{ns}/map   (global costmap live map source)
    #   /tf   → tf_relay       → /{ns}/tf    (map→odom transform for nav2)
    # ------------------------------------------------------------------
    if slam_val:
        rewritten_slam_params = rewrite_slam_params(slam_pf, ns)

        slam_node = Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name=f'slam_toolbox_{ns}',   # Distinguishable name for multi-robot
            # No namespace — see explanation above
            parameters=[rewritten_slam_params],
            output='screen',
        )
        actions.append(slam_node)

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