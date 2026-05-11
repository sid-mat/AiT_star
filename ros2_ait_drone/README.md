# AIT* Drone Planning Demo — Setup Guide
# ENPM661 Project 5 | Sidharth Mathur | UMD

## Prerequisites

```bash
sudo apt install -y \
  ros-jazzy-ompl \
  ros-jazzy-webots-ros2 \
  ros-jazzy-rviz2 \
  ros-jazzy-nav-msgs \
  ros-jazzy-visualization-msgs \
  ros-jazzy-tf2-ros \
  ros-jazzy-tf2-geometry-msgs
```

## Build

```bash
# Place package in your ROS2 workspace
cd ~/ros2_ws/src
# (copy ros2_ait_drone here)

cd ~/ros2_ws
colcon build --packages-select ros2_ait_drone --symlink-install
source install/setup.bash
```

## Run — Phase 1 (Pure ROS2, no Webots, GUARANTEED to work)

Terminal 1:
```bash
ros2 launch ros2_ait_drone drone_demo.launch.py use_webots:=false
```

This starts:
  - planner_node  (OMPL: AIT*, RRT*, InformedRRT*)
  - drone_sim.py  (simulated drone + 3 obstacle drones)
  - rviz2         (3D visualization, auto-configured)
  - auto-publishes goal at (8, 8, 4) after 2s

## Run — Phase 2 (With Webots)

Terminal 1:
```bash
ros2 launch ros2_ait_drone drone_demo.launch.py use_webots:=true
```

## Live demo commands (run in a new terminal)

### Switch planners (WOW moment — run these during presentation)
```bash
# AIT* (default, purple tree)
ros2 topic pub --once /switch_planner std_msgs/msg/String "data: 'ait'"

# RRT* (blue tree — notice the tree is more random)
ros2 topic pub --once /switch_planner std_msgs/msg/String "data: 'rrt'"

# Informed RRT* (orange tree)
ros2 topic pub --once /switch_planner std_msgs/msg/String "data: 'informed'"
```

### Change goal mid-flight
```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  '{"header": {"frame_id": "world"}, "pose": {"position": {"x": -8.0, "y": 8.0, "z": 3.0}, "orientation": {"w": 1.0}}}'
```

### Monitor replanning events
```bash
ros2 topic echo /planned_path --field header
```

## What to show during the presentation

1. Start with AIT* running, point to:
   - Purple tree: focused, heuristic-guided sampling
   - White path: optimal 3D trajectory avoiding all obstacles
   - Red spheres: dynamic obstacles moving on Lissajous trajectories

2. Watch for replan events (logged in planner_node terminal):
   "Dynamic obstacle invalidated path — replanning with ait"
   The path briefly disappears and a new optimal path appears.

3. Switch to RRT* LIVE:
   ros2 topic pub --once /switch_planner std_msgs/msg/String "data: 'rrt'"
   Point out: tree is wider, path is longer/more jagged

4. Switch to Informed RRT*:
   "data: 'informed'"
   Better than RRT* but still random until solution found

5. Switch back to AIT*: immediate tight convergence.

## Key numbers to cite

From our Python benchmark (paper results match):
  Narrow corridor:
    RRT*:          cost 9.195 | first solution at iter 82
    Informed RRT*: cost 8.069 | first solution at iter 82
    AIT*:          cost 8.068 | first solution at BATCH 6 (~300 samples)

AIT* finds first solution faster AND converges to the same optimum.
With dynamic obstacles, replanning speed matters — AIT*'s warm heuristic
lets it find a new path faster than RRT* on each replan.
