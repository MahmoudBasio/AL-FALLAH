# Your combined plant scan

`plant_combined_38.ply` combines all 38 captures from `plant_captures_38.zip`.
It contains 283,501 XYZ points in metres. It has no RGB colours.
The floor and distant room were cropped away; the pot remains.

## View all captures together in RViz

Download the PLY and `view_plant_model.py` into Downloads. Stop the old fusion
and manual capture programs. The Kinect driver is not needed for this saved model.

In terminal 1:

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=13
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
python3 ~/Downloads/view_plant_model.py ~/Downloads/plant_combined_38.ply
```

In terminal 2:

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=13
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
rviz2
```

Set Global Options → Fixed Frame to `plant_map`.
Add PointCloud2 → Topic `/plant_offline/model`.
Set Decay Time to 0, Size (m) to 0.003, Color Transformer to FlatColor.
Disable other point-cloud displays. Keep terminal 1 running. No TF is needed
when the fixed frame matches `plant_map`.

This is a saved combined model. Taking more captures does not update this file
automatically; they would need another alignment and fusion pass.

## What was done

The plant region was used for rigid multiscale trimmed ICP alignment. Adjacent
views, next-neighbor views, and three start/end connections formed a pose graph.
All 38 camera poses were jointly optimized with robust relative-pose residuals.
No view failed the chosen overlap thresholds. These thresholds are heuristics.

The plant and pot were then cropped above the estimated floor. Each view
contributed at most one mean per 5 mm voxel. Voxels supported by fewer than two
captures and spatially isolated points were removed. Repeated captures from
the same position can satisfy the two-capture rule; this is not independent
multi-angle confirmation. The lowest 25 mm near the floor was excluded.

The export axes are X forward, Y left, Z up relative to the initial camera,
with origin at that camera. This is an axis convention, not measured gravity.
The preview image shows front, top and side projections in the original camera
coordinate convention. Its colours represent height, not the plant's real RGB.

## Checks and limits

After optimization, neighboring-view median nearest-surface distances ranged
from about 1.4 to 9.1 mm; the median across pairs was about 6 mm. The last/first
view mismatch decreased from about 11 mm to 5 mm. These are fitting scores on
the input data, not independent measurements of accuracy.

The projections were visually inspected and the exported binary PLY was read
back and checked against the computed points. The RViz publisher's file reader
was exercised with this file; live ROS operation was not tested in this environment.

Residual leaf thickness, missing surfaces and local misalignment may remain.
The 5 mm grid does not imply 5 mm sensor accuracy. No missing leaves were
invented, and no watertight mesh was created. Inspect the model from multiple
angles before treating fine leaf or branch geometry as accurate.

`alignment_report.json` records the transforms, alignment scores and export
coordinate conversion. Your original captures have not been modified.
