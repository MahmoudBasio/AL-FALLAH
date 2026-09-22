# Demo 01 Architecture

This demo isolates the manipulation part of AL-FALLAH from the future mobile-base, perception, pruning, and physical-system stacks.

```text
Synthetic foliage point cloud
        ↓
User-defined target box
        ↓
Keep / excess classification
        ↓
Raster trimming path
        ↓
MoveIt 2 Cartesian planning
        ↓
TIAGo arm execution
        ↓
Virtual removal of contacted excess points
```

## Intentional boundaries

- The base remains stationary.
- The tree is synthetic and expressed in `base_footprint`.
- Only a front-face raster is demonstrated.
- TIAGo is a third-party simulation platform, not the AL-FALLAH robot.
- Physical-system development belongs under `/physical_system`, not in this demo.
