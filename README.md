# tb3_2_navigation2

Namespaced Nav2 + SLAM Toolbox bringup for one TurtleBot3 in a multi-robot
fleet, on ROS 2 Humble. Developed for NUS CDE2605R (Undergraduate Research
Experience), where up to four TurtleBot3 Burgers shared a network to study
multi-robot communication. The wider project lives in
[kenpegrasio/turtleswarm](https://github.com/kenpegrasio/turtleswarm).

The stock `turtlebot3_navigation2` launch assumes a single robot publishing
into the root namespace. With several robots on one network, every robot
needs its own `/<ns>/scan`, `/<ns>/odom`, `/<ns>/map` and `<ns>/...` TF
frames. This package makes that work without editing Nav2 or SLAM Toolbox.

## What it does

- **Rewrites parameters at launch.** `navigation2.launch.py` loads the Nav2
  params and prefixes every frame ID (`base_footprint`, `odom`, `map`, costmap
  frames), odom topic and costmap sensor topic with the robot namespace, then
  hands a rewritten temp file to `nav2_bringup`. The SLAM Toolbox params get
  absolute `/<ns>/...` topics and `<ns>/...` frames the same way.
- **Live SLAM mode (default).** SLAM Toolbox provides `map -> odom`. AMCL's TF
  broadcast is disabled so it cannot publish a competing transform. The global
  costmap's `static_layer` is removed and replaced with a fixed 20 m x 20 m
  window around the odom origin, which avoids "Robot is out of bounds" when
  odometry has drifted since the Pi booted.
- **Relays between root and namespaced topics.** SLAM Toolbox and the robot
  publish some topics at the root; the namespaced Nav2 stack listens under
  `/<ns>`. Four small `rclpy` relays bridge them:

  | Script | From -> to | Why not `topic_tools relay` |
  |---|---|---|
  | `tf_relay.py` | `/tf` -> `/<ns>/tf` | Queue depth 500 absorbs 50 Hz `map -> odom` bursts over Wi-Fi that otherwise get dropped, leaving the costmap stuck on "Checking transform" |
  | `tf_static_relay.py` | `/tf_static` -> `/<ns>/tf_static` | Accumulates all static transforms and republishes them `TRANSIENT_LOCAL`, so Nav2 nodes that start late still receive them |
  | `map_relay.py` | `/map` -> `/<ns>/map` | `TRANSIENT_LOCAL` buffering so a costmap that subscribes late still receives the latest map |
  | `map_updates_relay.py` | `/map_updates` -> `/<ns>/map_updates` | Forwards SLAM Toolbox's per-scan incremental updates so RViz's map does not freeze between full 2 s map publishes |

- **SLAM Toolbox runs without a node namespace on purpose.** Its params use
  absolute topics; adding a node namespace as well would resolve
  `/tb3_2/scan` to `/tb3_2/tb3_2/scan` and it would receive no scans.

## Usage

Prerequisites: ROS 2 Humble, `nav2_bringup`, `slam_toolbox`, and a TurtleBot3
bringup publishing `/<ns>/scan` and `/<ns>/odom` with `<ns>/`-prefixed frames.

```bash
cd ~/ros2_ws/src
git clone https://github.com/Nishuy52/tb3_2_navigation2.git
cd ~/ros2_ws && colcon build --packages-select tb3_2_navigation2
source install/setup.bash

export TURTLEBOT3_MODEL=burger
ros2 launch tb3_2_navigation2 navigation2.launch.py            # SLAM, no RViz
ros2 launch tb3_2_navigation2 navigation2.launch.py rviz:=True
ros2 launch tb3_2_navigation2 navigation2.launch.py slam:=False map:=/path/to/map.yaml
```

| Argument | Default | Meaning |
|---|---|---|
| `namespace` | `tb3_2` | Robot namespace applied to topics and frames |
| `slam` | `True` | Live mapping with SLAM Toolbox; `False` uses the static `map` with AMCL |
| `map` | `map/map.yaml` | Static map (always required by Humble's `map_server`, ignored in SLAM mode) |
| `params_file` | `param/humble/<model>.yaml` | Nav2 parameters before rewriting |
| `slam_params_file` | `param/slam_toolbox_params.yaml` | SLAM Toolbox parameters before rewriting |
| `rviz` | `False` | Off by default to save Wi-Fi bandwidth |
| `use_sim_time` | `false` | Use the Gazebo clock |

For another robot, pass a different namespace, e.g. `namespace:=tb3_1`.

## Credits

Based on ROBOTIS'
[turtlebot3_navigation2](https://github.com/ROBOTIS-GIT/turtlebot3) package
(Apache 2.0). TF fix contributed by
[@kenpegrasio](https://github.com/kenpegrasio).
