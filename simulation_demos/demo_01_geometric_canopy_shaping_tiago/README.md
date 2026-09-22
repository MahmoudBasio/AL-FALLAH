# Demo 01 — Geometric Canopy Shaping with TIAgO

> **Scope:** simulation proof of concept only.

This is the first public AL-FALLAH demo. Its purpose is to validate a small manipulation pipeline before mobile-base autonomy, physical-platform perception, pruning logic, and the custom AL-FALLAH hardware are introduced.

It is **not** the final AL-FALLAH simulation and is **not** the physical-platform software stack.

## What this demo is testing

The technical question is:

> Can a desired geometric canopy boundary be converted into a trimming trajectory that a manipulator can plan and execute in simulation?

The demo therefore focuses on:

1. generating a synthetic foliage point cloud;
2. defining a user-specified target box;
3. classifying points as retained or excess foliage;
4. generating a raster/boustrophedon trimming path on one target face;
5. using MoveIt 2 to compute and execute a Cartesian manipulator trajectory;
6. simulating cutting by removing contacted excess point-cloud samples.

## Why TIAgO is used

This demo uses the **TIAgO mobile manipulator from PAL Robotics** because it provides an existing ROS 2 / MoveIt-compatible platform for manipulation experiments.

TIAgO is a third-party robot design. It is **not** the AL-FALLAH robot, and no ownership of the TIAgO mechanical design, robot description, or PAL Robotics packages is claimed here.

The demo-specific code in this folder is AL-FALLAH research code built around that external platform. TIAgO packages themselves are not included in this repository.

## Deliberate simplifications

This demo intentionally does **not** implement:

- the custom AL-FALLAH mobile manipulator;
- autonomous base motion;
- physical depth-camera perception;
- branch-level pruning decisions;
- realistic cutting forces;
- tool dynamics;
- plant deformation;
- complete multi-face shaping;
- whole-body planning;
- final safety architecture.

Those limitations are intentional so that one subsystem can be validated at a time.

## Demo pipeline

```text
Synthetic foliage
      ↓
User-defined target geometry
      ↓
Keep / excess classification
      ↓
Raster trimming path
      ↓
MoveIt 2 Cartesian planning
      ↓
TIAgO arm execution
      ↓
Virtual excess-foliage removal
```

## Visualization

- Green — foliage inside the target geometry
- Red — excess foliage
- Blue/Cyan — target geometry
- Yellow — generated trimming trajectory

## Code in this demo

```text
ros2_ws/src/
├── tree_model/
│   ├── tree_generator.py
│   └── path_generator.py
└── trimming_executor/
    └── trimming_executor.cpp
```

## Running the demo

### Terminal 1 — TIAgO + MoveIt simulation

```bash
source /opt/ros/humble/setup.bash
source ~/tiago_public_ws/install/setup.bash
ros2 launch tiago_gazebo tiago_gazebo.launch.py moveit:=True is_public_sim:=True
```

### Terminal 2 — RViz

```bash
source /opt/ros/humble/setup.bash
source ~/tiago_public_ws/install/setup.bash
source <YOUR_DEMO_WS>/install/setup.bash
ros2 launch tiago_moveit_config moveit_rviz.launch.py
```

For this demo, use `base_footprint` as the RViz fixed frame and add:

- PointCloud2 `/tree/keep`
- PointCloud2 `/tree/cut`
- PointCloud2 `/tree/target_box`
- Marker `/tree/trimming_path`

### Terminal 3 — Synthetic foliage

```bash
ros2 run tree_model tree_generator --ros-args -p use_sim_time:=true
```

Example target dimensions:

```bash
ros2 run tree_model tree_generator --ros-args \
  -p use_sim_time:=true \
  -p box_width:=0.55 \
  -p box_depth:=0.55 \
  -p box_height:=1.00
```

### Terminal 4 — Trimming-path visualization

```bash
ros2 run tree_model path_generator --ros-args -p use_sim_time:=true
```

### Terminal 5 — Virtual trimming and MoveIt execution

```bash
ros2 topic pub --once /tree/cutting_enabled std_msgs/msg/Bool "{data: true}"
ros2 run trimming_executor trimming_executor --ros-args -p use_sim_time:=true
```

## Observed result

From the initial stationary base pose, the prototype reached approximately **78.9%** of the generated Cartesian trimming path. That limitation motivates a later demo for reachability-guided mobile-base repositioning rather than changing the tree or target geometry to fit the arm.

## Important frame limitation

The synthetic tree in this demo is expressed in `base_footprint`. This is acceptable only because the base does not move. A mobile-base demo must move tree/target geometry into a fixed world frame such as `odom` or `map`.

## What comes next

This folder remains an archived/traceable demo even as AL-FALLAH develops further. Later demos may cover mobile-base repositioning, sensor-based/simulated perception, pruning-specific planning, and multi-view reconstruction.
