# AL-FALLAH

**AL-FALLAH** is a developing research project for autonomous mobile manipulation in tree and vegetation maintenance.

The long-term project is broader than geometric tree shaping. Planned capabilities include canopy shaping, selective pruning, inspection, cutting, perception, autonomous base repositioning, and other tree-care operations.

> **Important:** The material currently published in this repository is **demo-stage research code**, not the final AL-FALLAH software stack and not the physical-system implementation.

## Repository purpose

This repository documents AL-FALLAH incrementally through a series of focused demonstrations. Each demo validates a specific technical idea before it is integrated into the complete system.

The demos are intentionally separated from development intended for deployment on the physical AL-FALLAH platform.

```text
AL-FALLAH/
├── simulation_demos/    # Isolated simulation experiments and proof-of-concept demos
├── physical_system/     # Reserved for physical-platform development
│   ├── software/
│   ├── mechanical_design/
│   └── electrical_design/
├── experiments/         # Research tests, prototypes, and technical investigations
└── docs/                # Project scope, architecture notes, and attribution
```

## Current demo

### Demo 01 — Geometric Canopy Shaping with a Stationary Manipulator

The first published demo validates a small part of the overall concept:

- synthetic foliage represented as a 3D point cloud;
- user-defined box target geometry;
- retained/excess foliage classification;
- raster trimming-path generation;
- Cartesian manipulator planning with MoveIt 2;
- simulated trimming through virtual point removal;
- RViz visualization of the process.

This demo deliberately keeps the mobile base stationary. It is **not** the final simulation and does not represent the complete autonomy architecture.

Future demos will investigate additional capabilities such as mobile-base repositioning, perception, pruning-oriented planning, multi-view reconstruction, and more realistic task constraints.

See [`simulation_demos/demo_01_geometric_canopy_shaping_tiago/`](simulation_demos/demo_01_geometric_canopy_shaping_tiago/) for the current demo.

## Third-party simulation platform

Demo 01 uses the **TIAGo mobile manipulator from PAL Robotics** as an external simulation platform. TIAGo is not the AL-FALLAH robot design, and this repository does not claim ownership of the TIAGo robot, its mechanical design, or PAL Robotics' simulation packages.

The TIAGo packages are treated as external dependencies and are not vendored into this repository. The demo-specific AL-FALLAH code uses that platform only to test perception/manipulation concepts before the project's own robot is integrated.

See [`docs/THIRD_PARTY.md`](docs/THIRD_PARTY.md).

## Simulation demos vs. physical-system development

### `simulation_demos/`
Contains isolated proofs of concept, simplified algorithms, synthetic data, simulator-specific assumptions, and temporary integrations with third-party robot models.

### `physical_system/`
Reserved for development intended for deployment on the physical AL-FALLAH platform. It is intentionally separated into:

- `software/` — physical-platform software (currently empty);
- `mechanical_design/` — CAD, assemblies, manufacturing documentation, and mechanical development (currently empty);
- `electrical_design/` — PCB design, schematics, wiring, power distribution, and electrical development (currently empty).

These directories are placeholders for later project stages. Current simulation-demo code should not be treated as physical-platform implementation.

## Project status

AL-FALLAH is at an early research and prototyping stage. The current public demo is only the beginning of the project and demonstrates one narrow capability.

Additional code, experiments, and implementations may exist outside the currently published demo and will be integrated when they are ready to be documented cleanly.

## Software used in Demo 01

- Ubuntu 22.04
- ROS 2 Humble
- MoveIt 2
- Gazebo Classic
- RViz2
- ros2_control
- Python 3 / NumPy / TF2
- PAL Robotics TIAGo simulation packages (external dependency)

## Development philosophy

```text
small isolated demo
        ↓
validate one technical idea
        ↓
measure limitations
        ↓
implement the next subsystem
        ↓
integrate on the custom robot
        ↓
physical-system validation
```

The purpose of the demo folders is to preserve the engineering progression of AL-FALLAH without confusing early simulation prototypes with the final system.
