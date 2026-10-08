# Incremental Kinect plant point cloud

This scanner builds **one accumulating XYZ model** from the Kinect's ROS `/points` stream. Each accepted capture is aligned to the existing scan and contributes measured points to the same model. New viewpoints can add coverage; repeated observations in the same voxel are averaged. Poor alignment is rejected so the existing model stays available.

The requested `--rate` is a maximum, not achieved throughput. Your P50 screenshot showed an accepted capture approximately every **3–5 seconds**, even with `--rate 3`. Move a few centimetres, hold still, and wait for the next `FUSED` message before moving again. The revised log includes processing time. Unchanged model arrays and RViz payloads are cached; live performance still needs checking on your laptop.

The plant and room must stay stationary. Move the camera, with substantial overlap between successive views. A fresh scan uses its first accepted camera view as `plant_map`. `--resume` retains an existing scan's coordinate system, crop and recorded poses.

## Start on your ThinkPad P50

Save `plant_fusion.py` in `~/Downloads`. Keep your working Kinect driver running in its terminal.

In a **new terminal**, paste only this code:

```bash
sudo apt update
sudo apt install python3-venv ros-humble-tf2-ros-py
/usr/bin/python3 -m venv --system-site-packages ~/kinect_fusion_env
~/kinect_fusion_env/bin/python -m pip install numpy==1.26.4 open3d==0.19.0
source /opt/ros/humble/setup.bash
source ~/kinect_ws/install/setup.bash
export ROS_DOMAIN_ID=13
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 ~/kinect_fusion_env/bin/python -X faulthandler ~/Downloads/plant_fusion.py --voxel 0.01 --max-voxels 750000 --rate 1
```

Use this environment's Python for subsequent runs too. The environment provides the tested Open3D version while keeping access to system ROS Python packages. The scanner requires Open3D 0.19 or newer and defaults to one native computation thread during initial validation. The documented installation pins Open3D 0.19.0 and NumPy 1.26.4.

At startup, a small registration check runs in a separate process before ROS scanning begins. It prints the package versions and `SELF-TEST PASSED`. If it crashes or fails, the scanner stops before starting a model and prints the available traceback. This is a check of synthetic registration, not a guarantee that live scanning is free of native crashes.

Before starting the scanner, position the Kinect so the entire plant and pot fit with margin. Start around 1.5 metres from the nearest foliage and adjust to fit. Centre the canopy vertically and horizontally. Leave some stationary room features visible to help camera tracking. Avoid waving foliage, people crossing the frame, and large camera jumps.

Use the same ROS domain, localhost and middleware settings in the driver, scanner and RViz terminals before starting each program. Keep your already working driver running. The command above starts a fresh scan with **10 mm processing voxels** and a 750,000-voxel cap. The script's defaults remain 5 mm and 500,000 voxels. These are processing settings, not sensor accuracy claims.

## Resume the scan already saved

Your screenshot showed **491,893 model points**, followed by `Voxel memory limit reached`. This was the prototype's 500,000-voxel cap; it does not establish that the laptop exhausted its physical RAM. Returning the camera cannot free that capacity. The revised scanner reports this separately and pauses new captures until you restart with a suitable setting.

Replace `~/Downloads/plant_fusion.py` with the updated script, using that exact filename. Keep the original scan folder. In the scanner terminal, paste only:

```bash
source /opt/ros/humble/setup.bash
source ~/kinect_ws/install/setup.bash
export ROS_DOMAIN_ID=13
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 ~/kinect_fusion_env/bin/python -X faulthandler ~/Downloads/plant_fusion.py --resume ~/plant_scan_20261007_201234_983485 --voxel 0.01 --max-voxels 750000 --rate 1
```

This reads `scan.json` and the accepted `captures/capture_*.npz` files, rebuilds the model at 10 mm voxels using the **saved poses**, and writes to a new scan folder. The original folder remains intact. Rebuilding prints progress every five captures and may take time on the P50. A PLY alone, or a scan made with `--no-record`, cannot be resumed this way. Only captures included in the last saved `scan.json` are restored; this is not recovery of unsaved records after a crash.

The saved model appears in RViz after rebuilding. Return the camera to a previously accepted view, preferably the first view, and hold still. Wait for `FUSED RECOVERED capture ...` before continuing. Recovery requires two fresh, consistent processed captures. The old saved pose is not broadcast as a new camera observation while waiting.

Rebuilding at a larger voxel size reduces detail and often reduces the number of occupied voxels. It preserves raw accepted measurements for a later finer rebuild. It **does not correct old camera poses**, remove room objects already inside the crop, or fix any existing tracking drift. Resume preserves the old crop; omit `--bounds` and `--no-record`. Moving the plant or rearranging the scene requires a fresh scan.

## Watch the model grow

For a ready camera view, save `plant_fusion_view.rviz` next to the script in `~/Downloads`. Keep the driver and scanner running, close the old RViz window, and launch a new one from a terminal using the same ROS settings as the driver and scanner. Our current laptop-only test uses domain 13 and Cyclone DDS:

```bash
source /opt/ros/humble/setup.bash
source ~/kinect_ws/install/setup.bash
export ROS_DOMAIN_ID=13
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
rviz2 -d ~/Downloads/plant_fusion_view.rviz
```

The preset selects the fused model with Reliable / Transient Local QoS, hides unrelated displays, and uses an Orbit view at distance 3.5 m centred on `(0, 0, 1.5)` in `plant_map`. Your latest screenshot shows the fused cloud in this preset. If a later display looks empty, check its topic status and viewing distance; a valid fixed frame alone does not establish cloud reception.

For a manual setup:

In RViz:

1. Set **Fixed Frame** to `plant_map`.
2. Add **PointCloud2**, with topic `/plant_fusion/model`.
3. Use **Points** with **Size (Pixels)** set to 3. This is a display setting.
4. Hide the raw `/points` display when inspecting the fused model.

Set **Reliability Policy** to **Reliable**, **Durability Policy** to **Transient Local**, and use a plain **FlatColor** colour transformer to begin. These match the model publisher. Expand the display's Status if nothing appears. The scanner keeps republishing the latest accepted model even while it waits for another capture or rejects a capture.

Keep the Fixed Frame at `plant_map`. After the first accepted capture, or after loading a saved model, this version publishes a static identity transform to `plant_scan_origin`, the first camera view, so RViz can recognize the model frame even if tracking pauses. It also publishes the estimated camera pose as `plant_map` → `plant_fusion_camera` at each newly accepted capture's timestamp. This dedicated camera frame uses the input XYZ cloud's axes and avoids changing the driver's own TF tree. Rejected captures and unconfirmed recovery candidates do not update the camera pose.

If you saw **“No tf data. Frame [plant_map] does not exist”** with an older script, use the updated scanner and install `ros-humble-tf2-ros-py` if needed. Keep the Kinect driver running. In RViz, disable RobotModel and the raw cloud display while checking the fused cloud. If the PointCloud2 Status is OK, adjust the 3D view to bring the cloud into view; the model is around Z = 1–2 m in the first camera's optical axes, where Y points downward. The scanner must remain running for a newly opened RViz to receive the live model.

Hold still until the terminal prints `FUSED capture 1`. Then move the Kinect a few centimetres at a time around the stationary plant, keeping most of the previous view visible. Start with a short arc across the front. Verify that the model stays in place while the camera moves before attempting a longer scan.

`FUSED capture ...` means that capture contributed to the model. `NOT_FUSED ...` means that capture was rejected. After two failed tracking attempts, the scanner searches near previously accepted camera poses. `RECOVERING ... hold still ...` asks you to keep the camera still for confirmation. `FUSED RECOVERED capture ...` means confirmation passed and fusion resumed in the same map.

Recovery keeps the first accepted view and up to 23 recent distinct views, searching at most two per processed capture. It uses stricter overlap and residual checks, plus a second pose-consistency check. This is **bounded recovery near known views**, not global relocalization from any new position. Return close to an accepted view and hold still while the search cycles. Ambiguous geometry and repeated foliage can still prevent recovery or produce a wrong match.

To explicitly request recovery while preserving the model, use another terminal with the same ROS settings:

```bash
ros2 service call /plant_fusion/relocalize std_srvs/srv/Trigger '{}'
```

This service cannot clear a model capacity limit. A capacity warning calls for saving and resuming with a larger voxel or cap, rather than moving the camera. `--max-voxels` accepts 10,000 to 2,000,000; a larger cap consumes more memory and processing time.

You can inspect status in another sourced terminal:

```bash
ros2 topic echo /plant_fusion/status
```

Status values describe registration residuals and overlap, not independently measured physical accuracy. These checks can miss an incorrect pose, particularly with repetitive foliage or moving objects.

## Crop region

The initial crop is a **manual box in the first camera frame**, in metres:

| Coordinate | Minimum | Maximum |
| --- | ---: | ---: |
| X, rightward | -0.7 | 0.7 |
| Y, downward | -1.1 | 1.1 |
| Z, forward depth | 0.7 | 2.5 |

The surrounding scene helps estimate camera motion, but only points inside this fixed region enter the published model. The crop is anchored to the first camera view and stays fixed as the camera moves. It is not plant recognition: any room, floor or other object inside the box can also enter the model. Points outside the box cannot be recovered in that live model merely by moving the camera.

If the whole plant does not fit this crop or too much room enters it, stop, adjust the camera or bounds, and start a new scan. For example, this changes the maximum forward depth to 3 m:

```bash
~/kinect_fusion_env/bin/python ~/Downloads/plant_fusion.py --bounds -0.7 0.7 -1.1 1.1 0.7 3.0
```

The six numbers are X minimum, X maximum, Y minimum, Y maximum, Z minimum and Z maximum. They describe the scan region, not a desired pruning shape.

## Save

Press **Ctrl+C** in the scanner terminal. It saves the model and prints its path. It also checkpoints after the first accepted capture and every ten accepted captures thereafter. Ctrl+Z suspends the process and does not finish saving.

To save without stopping, run in another sourced terminal:

```bash
ros2 service call /plant_fusion/save std_srvs/srv/Trigger '{}'
```

Each run creates a fresh `~/plant_scan_<date-and-time>/` folder containing:

- `plant_model.ply`: the current fused XYZ model in metres, with the number of capture observations per voxel.
- `scan.json`: crop, settings, acceptance counts and camera poses.
- `captures/capture_*.npz`: the accepted raw XYZ measurements and estimated poses, retained for later re-registration and model refinement.

The observation count is a count of contributions, not a confidence probability. Nearby frames can have correlated errors. Use `--resume` explicitly to load a saved scan; otherwise a fresh model is created.

## What this version does and what still needs validation

This is a CPU prototype using Open3D multiscale ICP for camera tracking and per-capture voxel averaging for the point model. It uses geometry only, so it does not rely on the current driver's incomplete RGB timestamps or RGB/depth registration. It retains observed surfaces and does not invent missing leaves, branches or the unseen back of the plant.

The fusion core was checked with known synthetic camera transformations, incremental coverage, repeated measured Kinect data, bad-overlap rejection, an ambiguous plane, binary field layouts, recording, PLY round trips and the ROS adapter with mocked messages. The user then reported a segmentation fault after the first fused live capture with the original setup. The short log does not identify the failing native call. The earlier installation command used Ubuntu's Open3D package, whereas the initial local checks used Open3D 0.19.0; the revised instructions correct that mismatch. Revised checks use Open3D 0.19.0 with NumPy 1.26.4 and one native computation thread.

The update also copies native transformation matrices into owned arrays, retains the robust-kernel object during ICP, enables Python fault tracing, checks registration before scanning, saves the first model, and republishes the latest cloud while waiting. Your latest P50 screenshot shows 40 accepted captures and a visible RViz cloud, followed by tracking and capacity warnings.

The recovery revision was tested offline with a synthetic 24 cm return to the first view: normal tracking rejected the jump, a candidate left the model unchanged, and a second consistent capture resumed fusion. Unrelated geometry, a featureless plane and movement during confirmation remained rejected. Capacity rejection preserved model points, observation counts and the accepted pose. A saved synthetic scan was rebuilt at coarser voxels with the stored poses, and its original files remained intact. Original recorded Kinect bytes were also used for repeated-view registration and the mocked ROS adapter. Those adapter checks include recovery services, duplicate timestamps, cached cloud data, persistent origin TF on resume and no live camera TF before confirmation. **The new recovery and resume behavior has not yet been tested live on the P50**, and no independently measured real multi-view accuracy is established.

There is **no loop closure or pose-graph refinement** in this first version. A long orbit can accumulate camera-tracking drift. Begin with a short arc and inspect for doubled leaves or a moving/smeared plant. Full-circle accuracy, the sensor's true spatial accuracy, calibration and performance on the P50 remain unverified. The 5 mm voxel setting is not a claim of 5 mm sensor accuracy. Getting the best useful model requires reliable poses, stationary foliage, sufficient viewpoints and verified calibration.

The script keeps raw accepted geometry so a later refinement stage can correct poses and rebuild the model instead of being limited to the fused output alone.

## Technical sources

- [Open3D ICP registration](https://www.open3d.org/docs/release/tutorial/pipelines/icp_registration.html): point-to-point and point-to-plane registration, normals and alignment metrics.
- [Open3D multiway registration](https://www.open3d.org/docs/release/tutorial/pipelines/multiway_registration.html): the pose-graph refinement approach for a later stage.
- [Ubuntu package catalogue: python3-open3d](https://packages.ubuntu.com/search?keywords=python3-open3d): Jammy package availability.
- [Python virtual environments](https://docs.python.org/3/library/venv.html): an isolated environment with access to system site packages.
- [Python fault tracing](https://docs.python.org/3/library/faulthandler.html): traceback reporting for segmentation faults.
- [ROS Python TF bindings](https://index.ros.org/p/tf2_ros_py/): the `tf2_ros_py` package used to broadcast the scan origin and camera transforms.

## If it still crashes or RViz is empty

Keep the Kinect driver running. For a crash, copy the `[Runtime]` line, the last `[Native check]` line, and the traceback printed by the revised script. This identifies the active interpreter, package versions and Python call site at the fault. The check cannot establish the precise C++ root cause on its own.

For an empty RViz display, first verify that the scanner is still running and has printed `FUSED capture ...`. In a new sourced terminal, run:

```bash
ros2 topic info /plant_fusion/model --verbose
ros2 topic echo /plant_fusion/model --once --field header --qos-reliability reliable --qos-durability transient_local
ros2 topic echo /plant_fusion/model --once --field width --qos-reliability reliable --qos-durability transient_local
```

Use the same ROS environment as the driver, scanner and RViz for these commands. The header command should print `frame_id: plant_map`, proving that this terminal received a model message. The width command prints the number of points in this scanner's one-row cloud and should be greater than zero. If an echo command remains waiting for 10 seconds, stop it with Ctrl+C and copy the topic-info output for diagnosis. Avoid echoing the entire cloud's binary data.

Use Fixed Frame `plant_map`, PointCloud2 topic `/plant_fusion/model`, matching QoS as above, and inspect the display Status. A ROS topic publisher alone does not prove that usable messages have reached RViz. The TF warning and cloud reception are separate checks: a cloud already expressed in the fixed frame does not need a transform to that same frame.
