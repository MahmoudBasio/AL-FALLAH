#!/usr/bin/env python3
"""Incremental geometric plant scanner for Kinect v1 / ROS 2 Humble.

Keep the plant and room still, centre the complete plant in the first view,
then move the camera slowly with generous overlap. Uses /points only; RGB
registration is not required. Output /plant_fusion/model is in plant_map,
the coordinate system of the first accepted camera view. A fixed manual
3D crop selects the plant area, not a semantic plant detector.

Use Open3D 0.19.0 in a system-Python virtual environment; see the setup guide.
Run after sourcing ROS and the driver workspace:
    ~/kinect_fusion_env/bin/python plant_fusion.py

RViz: Fixed Frame plant_map; add PointCloud2 /plant_fusion/model.
Save while running:
    ros2 service call /plant_fusion/save std_srvs/srv/Trigger '{}'
Ctrl+C also saves. Use --help for crop bounds and offline ZIP replay.

CPU prototype: multiscale ICP tracking plus per-capture voxel averaging.
No loop closure, global relocalization, RGB colour, or missing-surface
completion. Quality gates are heuristics, not proof of a correct pose.
Voxel size is a processing setting, not calibrated sensor accuracy.
"""

import os
os.environ.setdefault("OMP_NUM_THREADS", "1")
os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")

import argparse
from collections import deque
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import re
import shutil
import subprocess
import sys
import time
import zipfile
import faulthandler

faulthandler.enable(all_threads=True)

import numpy as np
try:
    import open3d as o3d
except ImportError:
    raise SystemExit("Install Open3D 0.19.0 in ~/kinect_fusion_env; use the updated Plant_Fusion_Setup.md")

version_numbers = tuple(int(x) for x in re.findall(r"\d+", o3d.__version__)[:3])
if version_numbers < (0, 19, 0):
    raise SystemExit(f"Open3D {o3d.__version__} is outside this scanner's supported setup. "
                     "Install open3d==0.19.0 in ~/kinect_fusion_env, then run "
                     "~/kinect_fusion_env/bin/python ~/Downloads/plant_fusion.py")


def decode_xyz(data, width, height, point_step, row_step, fields, bigendian=False):
    """Read original XYZ fields with endian and row padding preserved."""
    if width < 1 or height < 1 or point_step < 1 or row_step < width * point_step:
        raise ValueError("Invalid PointCloud2 dimensions or strides")
    if len(data) < height * row_step:
        raise ValueError("Truncated PointCloud2 payload")
    names = {f["name"]: f for f in fields}
    channels = []
    for name in ("x", "y", "z"):
        field = names.get(name)
        if field is None or field.get("count", 1) != 1 or field["datatype"] not in (7, 8):
            raise ValueError("XYZ must be scalar FLOAT32 or FLOAT64 fields")
        size = 4 if field["datatype"] == 7 else 8
        if field["offset"] < 0 or field["offset"] + size > point_step:
            raise ValueError("XYZ field lies outside the point record")
        dtype = (">" if bigendian else "<") + ("f4" if size == 4 else "f8")
        channels.append(np.ndarray((height, width), dtype=dtype, buffer=data,
                                  offset=field["offset"], strides=(row_step, point_step)))
    return np.stack(channels, axis=-1).reshape(-1, 3).astype(np.float64)


def read_capture(path):
    with zipfile.ZipFile(path) as archive:
        meta = json.loads(archive.read("metadata.json"))
        p = meta["streams"]["points"]
        xyz = decode_xyz(archive.read("points.bin"), p["width"], p["height"],
                         p["point_step"], p["row_step"], p["fields"], p["is_bigendian"])
    return xyz, p.get("frame_id", "kinect_depth")


def cloud(xyz):
    result = o3d.geometry.PointCloud()
    result.points = o3d.utility.Vector3dVector(np.ascontiguousarray(xyz, dtype=np.float64))
    return result


def transform(xyz, pose):
    return xyz @ pose[:3, :3].T + pose[:3, 3]


def rotation_quaternion(rotation):
    """Convert a proper rotation matrix to normalized ROS (x, y, z, w)."""
    r = np.asarray(rotation, dtype=np.float64)
    if r.shape != (3, 3) or not np.isfinite(r).all():
        raise ValueError("Expected a finite 3 x 3 rotation")
    trace = float(np.trace(r))
    if trace > 0:
        s = 2.0 * math.sqrt(trace + 1.0)
        q = [(r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s,
             (r[1, 0] - r[0, 1]) / s, s / 4.0]
    else:
        i = int(np.argmax(np.diag(r)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = 2.0 * math.sqrt(max(1.0 + r[i, i] - r[j, j] - r[k, k], 0.0))
        if s <= 1e-12:
            raise ValueError("Invalid camera rotation")
        q = np.zeros(4)
        q[i], q[j] = s / 4.0, (r[j, i] + r[i, j]) / s
        q[k], q[3] = (r[k, i] + r[i, k]) / s, (r[k, j] - r[j, k]) / s
    q = np.asarray(q, dtype=np.float64)
    return q / np.linalg.norm(q)


def crop_mask(xyz, bounds):
    low = np.asarray(bounds)[[0, 2, 4]]
    high = np.asarray(bounds)[[1, 3, 5]]
    return np.all((xyz >= low) & (xyz <= high), axis=1)


class VoxelCapacityError(ValueError):
    pass


class VoxelModel:
    """One centroid observation per occupied voxel per accepted capture."""
    def __init__(self, size=0.005, limit=500000):
        self.size, self.limit = size, limit
        self.cells = {}
        self._cached_arrays = None

    def integrate(self, xyz):
        if not len(xyz):
            return
        keys, inverse = np.unique(np.floor(xyz / self.size).astype(np.int64), axis=0,
                                  return_inverse=True)
        count = np.bincount(inverse)
        sums = np.zeros((len(keys), 3))
        np.add.at(sums, inverse, xyz)
        means = sums / count[:, None]
        tuples = [tuple(k) for k in keys]
        if len(self.cells) + sum(k not in self.cells for k in tuples) > self.limit:
            raise VoxelCapacityError(f"Model capacity reached ({self.limit:,} voxels); captures paused. "
                                     "Save and resume with a larger --voxel or --max-voxels")
        # Preflight the capacity check before mutating any cell.
        self._cached_arrays = None
        for key, mean in zip(tuples, means):
            if key in self.cells:
                previous, observations = self.cells[key]
                self.cells[key] = ((previous * observations + mean) / (observations + 1),
                                   observations + 1)
            else:
                self.cells[key] = (mean.copy(), 1)

    def arrays(self):
        if self._cached_arrays is None:
            points = (np.stack([v[0] for v in self.cells.values()]) if self.cells
                      else np.empty((0, 3)))
            observations = np.asarray([v[1] for v in self.cells.values()], dtype=np.uint32)
            points.setflags(write=False)
            observations.setflags(write=False)
            self._cached_arrays = points, observations
        return self._cached_arrays


def pose_distance(first, second):
    relative = np.linalg.inv(first) @ second
    distance = float(np.linalg.norm(relative[:3, 3]))
    angle = math.degrees(math.acos(float(np.clip((np.trace(relative[:3, :3]) - 1) / 2, -1, 1))))
    return distance, angle


def valid_pose(pose):
    return (pose.shape == (4, 4) and np.isfinite(pose).all()
            and np.allclose(pose[3], [0, 0, 0, 1], atol=1e-6)
            and np.allclose(pose[:3, :3].T @ pose[:3, :3], np.eye(3), atol=1e-5)
            and abs(np.linalg.det(pose[:3, :3]) - 1.0) < 1e-5)


def alignment(source, target, initial, trace=False):
    """Estimate current-camera -> first-camera pose, with a local initialization."""
    reg = o3d.pipelines.registration
    pose = np.array(initial, dtype=np.float64, order="C", copy=True)
    for voxel, distance, iterations in ((0.06, 0.15, 25), (0.025, 0.045, 35)):
        if trace:
            print(f"[Native check] downsample {voxel:g} m", flush=True)
        src = source.voxel_down_sample(voxel)
        dst = target.voxel_down_sample(voxel)
        if len(src.points) < 100 or len(dst.points) < 100:
            return None, {"reason": "Too little measured geometry for tracking"}
        if voxel == 0.06:
            estimator = reg.TransformationEstimationPointToPoint()
        else:
            if trace:
                print("[Native check] estimate normals", flush=True)
            dst.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.10, max_nn=35))
            kernel = reg.HuberLoss(k=0.015)
            estimator = reg.TransformationEstimationPointToPlane(kernel)
        if trace:
            print(f"[Native check] ICP {voxel:g} m", flush=True)
        result = reg.registration_icp(src, dst, distance, pose, estimator,
                                     reg.ICPConvergenceCriteria(max_iteration=iterations))
        # Own the matrix before the previous native result is released.
        pose = np.array(result.transformation, dtype=np.float64, order="C", copy=True)
    if not np.isfinite(pose).all():
        return None, {"reason": "Non-finite camera pose"}
    if trace:
        print("[Native check] evaluate alignment", flush=True)
    inverse_score = reg.evaluate_registration(dst, src, 0.045, np.ascontiguousarray(np.linalg.inv(pose)))
    pairs = np.asarray(result.correspondence_set)
    # A single featureless plane cannot constrain all camera motions.
    conditioning = 0.0
    if len(pairs) >= 100:
        p = transform(np.asarray(src.points)[pairs[:, 0]], pose)
        n = np.asarray(dst.normals)[pairs[:, 1]]
        p = p - p.mean(axis=0)
        p /= max(float(np.sqrt(np.mean(np.sum(p * p, axis=1)))), 0.1)
        j = np.hstack((np.cross(p, n), n))
        eigenvalues = np.linalg.eigvalsh(j.T @ j / len(j))
        conditioning = float(max(eigenvalues[0], 0.0) / max(eigenvalues[-1], 1e-12))
    relative = np.linalg.inv(initial) @ pose
    # Do not let machine roundoff move an unchanged scan across voxel faces.
    if np.allclose(relative, np.eye(4), atol=1e-10, rtol=0.0):
        pose = np.array(initial, dtype=np.float64, order="C", copy=True)
        relative = np.eye(4)
    step = float(np.linalg.norm(relative[:3, 3]))
    angle = math.degrees(math.acos(float(np.clip((np.trace(relative[:3, :3]) - 1) / 2, -1, 1))))
    metrics = {"fitness": float(result.fitness), "reverse_fitness": float(inverse_score.fitness),
               "rmse_m": float(result.inlier_rmse), "camera_step_m": step,
               "camera_step_deg": angle, "geometry_condition": conditioning}
    if not valid_pose(pose):
        reason = "Invalid rotation"
    elif result.fitness < 0.50 or inverse_score.fitness < 0.30:
        reason = "Not enough scan overlap"
    elif result.inlier_rmse > 0.020:
        reason = "Alignment residual is too large"
    elif step > 0.15 or angle > 15.0:
        reason = "Camera moved too far between processed captures"
    elif conditioning < 0.0001:
        reason = "Geometry is too ambiguous to track; include room corners and the pot"
    else:
        return pose, metrics
    return None, {**metrics, "reason": reason}


class FusionEngine:
    def __init__(self, bounds, voxel=0.005, output=None, record=True, max_voxels=500000):
        self.bounds = list(bounds)
        self.model = VoxelModel(voxel, max_voxels)
        self.pose = np.eye(4)
        self.references = deque(maxlen=3)
        self.keyframes = []
        self.tracking_failures = 0
        self.recovery_requested = False
        self.recovery_pending = None
        self.recovery_cursor = 0
        self.capacity_blocked = False
        self.resumed_from = None
        self.accepted, self.rejected = 0, 0
        self.frame_id = None
        self.output = Path(output) if output is not None else None
        self.record = record
        self.trajectory = []
        self.last_status = {"state": "waiting"}
        if self.output is not None:
            self.output.mkdir(parents=True, exist_ok=False)
            if record:
                (self.output / "captures").mkdir()

    def reject(self, reason, **metrics):
        self.rejected += 1
        state = "recovering" if metrics.get("code", "").startswith("recovery_") else "not_fused"
        self.last_status = {"state": state, "reason": reason, **metrics,
                            "accepted_captures": self.accepted, "rejected_captures": self.rejected,
                            "model_points": len(self.model.cells)}
        return self.last_status

    def remember_view(self, source, pose, capture):
        scene = transform(np.asarray(source.points), pose)
        self.references.append(scene)
        if self.keyframes:
            distance, angle = pose_distance(self.keyframes[-1]["pose"], pose)
            if distance < 0.06 and angle < 5.0:
                return
        if len(self.keyframes) == 24:
            del self.keyframes[1]  # Keep the first view and 23 recent distinct views.
        self.keyframes.append({"capture": capture, "pose": pose.copy(), "scene": scene})

    def request_recovery(self):
        if not self.accepted:
            raise ValueError("Capture the first view before requesting recovery")
        if self.capacity_blocked:
            raise ValueError("Model capacity reached; save and resume with a larger voxel or limit")
        self.recovery_requested = True
        self.recovery_pending = None
        self.recovery_cursor = 0

    @staticmethod
    def strong_recovery(metrics):
        return (metrics.get("fitness", 0) >= 0.70
                and metrics.get("reverse_fitness", 0) >= 0.50
                and metrics.get("rmse_m", math.inf) <= 0.015
                and metrics.get("geometry_condition", 0) >= 0.0001)

    def recover(self, source):
        """Search only near accepted poses; confirm on a second fresh capture."""
        if self.recovery_pending is not None:
            pending = self.recovery_pending
            anchor = pending["anchor"]
            pose, metrics = alignment(source, cloud(anchor["scene"]), pending["pose"])
            if pose is not None and self.strong_recovery(metrics):
                distance, angle = pose_distance(pending["pose"], pose)
                if distance <= 0.025 and angle <= 2.0:
                    return pose, {**metrics, "relocalized": True,
                                  "reference_capture": anchor["capture"],
                                  "recovery_confirmation_m": distance,
                                  "recovery_confirmation_deg": angle}
            self.recovery_pending = None
            return None, {"reason": "Recovery confirmation failed; return to a previously accepted view and hold still",
                          "code": "recovery_search"}

        # Limit expensive searches to two saved views per processed capture.
        anchors = [self.keyframes[0], *reversed(self.keyframes[1:])]
        for _ in range(min(2, len(anchors))):
            anchor = anchors[self.recovery_cursor % len(anchors)]
            self.recovery_cursor += 1
            pose, metrics = alignment(source, cloud(anchor["scene"]), anchor["pose"])
            if pose is None or not self.strong_recovery(metrics):
                continue
            self.recovery_pending = {"anchor": anchor, "pose": pose.copy()}
            return None, {**metrics, "reason": "Matched an accepted view; hold still for a second capture to confirm",
                          "code": "recovery_pending", "reference_capture": anchor["capture"]}
        return None, {"reason": "Searching accepted views; return to a previously accepted view and hold still",
                      "code": "recovery_search", "saved_views": len(anchors)}

    def update(self, xyz, frame_id="kinect_depth", stamp=None):
        if self.capacity_blocked:
            return self.reject("Model capacity reached; captures paused. Save and resume with a larger --voxel or --max-voxels",
                               code="voxel_limit", max_voxels=self.model.limit)
        xyz = np.asarray(xyz, dtype=np.float64)
        if xyz.ndim != 2 or xyz.shape[1] != 3:
            raise ValueError("Expected an N x 3 array")
        xyz = xyz[np.isfinite(xyz).all(axis=1) & (xyz[:, 2] >= 0.5) & (xyz[:, 2] <= 4.5)]
        if len(xyz) < 1000:
            return self.reject("Too few finite depth measurements")
        if not frame_id:
            return self.reject("Point cloud has an empty frame_id")
        if self.frame_id is not None and frame_id != self.frame_id:
            return self.reject("Input frame_id changed during the scan")
        source = cloud(xyz).voxel_down_sample(0.015)
        metrics = {}
        if self.recovery_requested:
            pose, metrics = self.recover(source)
            if pose is None:
                return self.reject(**metrics)
        elif self.references:
            # Use matching sampling on both sides in alignment(); an extra
            # target-only voxel pass biases an otherwise identical capture.
            target = cloud(np.vstack(self.references))
            pose, metrics = alignment(source, target, self.pose, trace=(self.accepted == 1))
            if pose is None:
                self.tracking_failures += 1
                if self.tracking_failures >= 2:
                    self.request_recovery()
                    pose, metrics = self.recover(source)
                if pose is None:
                    return self.reject(**metrics)
        else:
            pose = np.eye(4)
        world = transform(xyz, pose)
        selected = world[crop_mask(world, self.bounds)]
        if len(selected) < 100:
            return self.reject("Very little geometry is inside the plant crop; check --bounds", **metrics)
        try:
            self.model.integrate(selected)
        except VoxelCapacityError as error:
            self.capacity_blocked = True
            return self.reject(str(error), **metrics, code="voxel_limit", max_voxels=self.model.limit)
        self.pose, self.frame_id = pose.copy(), frame_id
        if metrics.get("relocalized"):
            self.references.clear()
        self.tracking_failures = 0
        self.recovery_requested = False
        self.recovery_pending = None
        self.accepted += 1
        self.remember_view(source, pose, self.accepted)
        self.trajectory.append({"capture": self.accepted, "stamp": stamp,
                                "camera_to_map": pose.tolist(), "metrics": metrics})
        if self.output is not None and self.record:
            # Preserve measurements and poses for later re-registration/refinement.
            np.savez_compressed(self.output / "captures" / f"capture_{self.accepted:06d}.npz",
                                xyz=xyz.astype(np.float32), camera_to_map=pose,
                                frame_id=np.asarray(frame_id))
        self.last_status = {"state": "fused", "accepted_captures": self.accepted,
                            "rejected_captures": self.rejected, "model_points": len(self.model.cells),
                            "points_in_crop": len(selected), **metrics}
        return self.last_status

    @classmethod
    def resume(cls, directory, voxel, output, max_voxels=500000):
        """Rebuild with saved measurements/poses; leave the original scan intact."""
        directory = Path(directory).expanduser().resolve()
        try:
            details = json.loads((directory / "scan.json").read_text())
            if details["format"] != "kinect_incremental_voxel_fusion_v1":
                raise ValueError("Unrecognized scan format")
            total = details["accepted_captures"]
            trajectory = details["trajectory"]
            bounds = details["bounds_xyz"]
            frame_id = details["input_frame"]
            if (not isinstance(total, int) or total < 1 or len(trajectory) != total
                    or len(bounds) != 6 or not all(math.isfinite(v) for v in bounds)
                    or any(bounds[i] >= bounds[i + 1] for i in (0, 2, 4))
                    or not isinstance(frame_id, str) or not frame_id):
                raise ValueError("Invalid scan metadata")
            paths = [directory / "captures" / f"capture_{i:06d}.npz" for i in range(1, total + 1)]
            if not all(path.is_file() for path in paths):
                raise ValueError("Resume needs every accepted captures/capture_*.npz file; PLY alone is insufficient")
        except (KeyError, TypeError, json.JSONDecodeError) as error:
            raise ValueError(f"Invalid scan metadata in {directory}") from error
        output = Path(output).expanduser().resolve()
        if output == directory or directory in output.parents:
            raise ValueError("Choose a fresh output folder outside the original scan folder")
        engine = cls(bounds, voxel, output, True, max_voxels)
        engine.resumed_from = str(directory)
        print(f"Rebuilding {total} saved captures at {voxel:g} m voxels, using their saved poses and crop...", flush=True)
        for i, path in enumerate(paths, 1):
            with np.load(path, allow_pickle=False) as record:
                xyz = np.asarray(record["xyz"], dtype=np.float64)
                pose = np.asarray(record["camera_to_map"], dtype=np.float64)
                saved_frame = str(record["frame_id"].item())
            entry = trajectory[i - 1]
            if (xyz.ndim != 2 or xyz.shape[1] != 3 or not valid_pose(pose)
                    or saved_frame != frame_id or entry["capture"] != i
                    or not np.allclose(pose, np.asarray(entry["camera_to_map"]), atol=1e-8)):
                raise ValueError(f"Invalid saved capture or pose: {path.name}")
            xyz = xyz[np.isfinite(xyz).all(axis=1) & (xyz[:, 2] >= 0.5) & (xyz[:, 2] <= 4.5)]
            world = transform(xyz, pose)
            engine.model.integrate(world[crop_mask(world, bounds)])
            engine.pose, engine.frame_id = pose.copy(), frame_id
            engine.accepted = i
            engine.trajectory.append(entry)
            engine.remember_view(cloud(xyz).voxel_down_sample(0.015), pose, i)
            destination = engine.output / "captures" / path.name
            try:
                os.link(path, destination)  # Saved captures are immutable.
            except OSError:
                shutil.copy2(path, destination)
            if i % 5 == 0 or i == total:
                print(f"Rebuilt {i}/{total}: {len(engine.model.cells):,} model points", flush=True)
        engine.rejected = int(details.get("rejected_captures", 0))
        engine.request_recovery()
        engine.last_status = {"state": "recovering", "code": "recovery_search",
                              "reason": "Saved model loaded; return to an accepted view and hold still",
                              "accepted_captures": engine.accepted, "model_points": len(engine.model.cells)}
        engine.save()
        return engine

    def save(self):
        if self.output is None:
            raise ValueError("No output directory configured")
        if not self.accepted:
            raise ValueError("No accepted capture to save")
        points, counts = self.model.arrays()
        vertices = np.empty(len(points), dtype=[("x", "<f4"), ("y", "<f4"),
                                               ("z", "<f4"), ("observations", "<u4")])
        for k, name in enumerate(("x", "y", "z")):
            vertices[name] = points[:, k]
        vertices["observations"] = counts
        header = ("ply\nformat binary_little_endian 1.0\n"
                  "comment XYZ metres; plant_map is the first accepted camera frame\n"
                  "comment Manual fixed crop; observed surfaces only; no loop closure\n"
                  f"element vertex {len(points)}\nproperty float x\nproperty float y\n"
                  "property float z\nproperty uint observations\nend_header\n").encode()
        destination = self.output / "plant_model.ply"
        temporary = destination.with_suffix(".ply.tmp")
        with temporary.open("wb") as handle:
            handle.write(header)
            handle.write(vertices.tobytes())
        temporary.replace(destination)
        details = {"format": "kinect_incremental_voxel_fusion_v1", "saved_utc": datetime.now(timezone.utc).isoformat(),
                   "input_frame": self.frame_id, "map_frame": "plant_map", "units": "metres",
                   "bounds_xyz": self.bounds, "voxel_size_m": self.model.size,
                   "max_voxels": self.model.limit, "resumed_from": self.resumed_from,
                   "accepted_captures": self.accepted, "rejected_captures": self.rejected,
                   "model_points": len(points), "trajectory": self.trajectory,
                   "limitations": ["Manual box crop, not semantic segmentation",
                                   "ICP quality gates can miss incorrect alignment",
                                   "Bounded known-view recovery only; no global relocalization or loop closure",
                                   "No RGB registration or invented hidden surfaces",
                                   "Voxel size does not establish physical accuracy"]}
        temporary = self.output / "scan.json.tmp"
        temporary.write_text(json.dumps(details, indent=2, allow_nan=False))
        temporary.replace(self.output / "scan.json")
        return destination


def parser():
    result = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    result.add_argument("--topic", default="/points")
    result.add_argument("--rate", type=float, default=1.0, help="Maximum processed captures/second; latest frame only")
    result.add_argument("--voxel", type=float, default=0.005, help="Fusion voxel size in metres; not sensor accuracy")
    result.add_argument("--max-voxels", type=int, default=500000,
                        help="Model capacity; default 500000, configurable up to 2000000")
    result.add_argument("--bounds", nargs=6, type=float,
                        metavar=("XMIN", "XMAX", "YMIN", "YMAX", "ZMIN", "ZMAX"),
                        help="Fixed crop in the FIRST camera frame, metres; adjust to fit your plant and exclude the room")
    result.add_argument("--output", type=Path, help="New scan directory; existing directories are refused")
    result.add_argument("--no-record", action="store_true", help="Omit compressed raw XYZ records; model is still saved")
    modes = result.add_mutually_exclusive_group()
    modes.add_argument("--replay", nargs="+", type=Path, help="Offline: process capture ZIPs in supplied order, without ROS")
    modes.add_argument("--resume", type=Path, help="Rebuild a saved scan's raw captures at this voxel size, then resume live scanning")
    modes.add_argument("--self-test", action="store_true", help="Check native ICP in this environment without ROS or a camera")
    return result


def create_engine(args, output):
    if args.resume:
        return FusionEngine.resume(args.resume, args.voxel, output, args.max_voxels)
    bounds = args.bounds or [-0.7, 0.7, -1.1, 1.1, 0.7, 2.5]
    return FusionEngine(bounds, args.voxel, output, not args.no_record, args.max_voxels)


def self_test():
    """Small known-pose check that executes the native registration stages."""
    print(f"[Runtime] Python {sys.version.split()[0]} | Open3D {o3d.__version__} | NumPy {np.__version__}", flush=True)
    rng = np.random.default_rng(14)
    uv = rng.uniform(-0.7, 0.7, (1800, 2))
    scene = np.vstack((np.column_stack((uv, 2.0 + 0.08 * np.sin(7 * uv[:, 0]))),
                       np.column_stack((np.full(len(uv), -0.8), uv[:, 0], 1.4 + uv[:, 1])),
                       np.column_stack((uv[:, 0], np.full(len(uv), 0.8), 1.4 + uv[:, 1]))))
    expected = np.eye(4)
    expected[:3, :3] = o3d.geometry.get_rotation_matrix_from_xyz((0.01, 0.025, -0.01))
    expected[:3, 3] = [0.02, -0.01, 0.015]
    current = transform(scene, np.linalg.inv(expected))
    estimated, metrics = alignment(cloud(current), cloud(scene), np.eye(4), trace=True)
    if estimated is None:
        raise RuntimeError(f"Native self-test rejected known motion: {metrics}")
    relative = np.linalg.inv(expected) @ estimated
    error_m = float(np.linalg.norm(relative[:3, 3]))
    error_deg = math.degrees(math.acos(float(np.clip((np.trace(relative[:3, :3]) - 1) / 2, -1, 1))))
    if error_m > 0.005 or error_deg > 0.5:
        raise RuntimeError(f"Native self-test did not recover known motion: {error_m:g} m, {error_deg:g} deg")
    print("SELF-TEST PASSED", flush=True)


def native_preflight():
    """Probe native code in a child so a failed check does not start a scan."""
    print("Checking registration before starting the scan...", flush=True)
    try:
        result = subprocess.run([sys.executable, "-X", "faulthandler", str(Path(__file__).resolve()), "--self-test"],
                                capture_output=True, text=True, timeout=60)
    except subprocess.TimeoutExpired as error:
        raise RuntimeError("Native registration check timed out; no scan was started") from error
    if result.stdout:
        print(result.stdout, end="", flush=True)
    if result.returncode != 0:
        if result.stderr:
            print(result.stderr, end="", file=sys.stderr, flush=True)
        raise RuntimeError(f"Native registration check failed (exit {result.returncode}); "
                           "no scan was started. Copy the runtime and traceback lines for diagnosis")


def run_ros(args, output):
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.executors import ExternalShutdownException
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        from sensor_msgs.msg import PointCloud2, PointField
        from geometry_msgs.msg import TransformStamped
        from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
        from std_msgs.msg import String
        from std_srvs.srv import Trigger
    except ImportError as error:
        raise RuntimeError(f"ROS import failed: {error}. Source /opt/ros/humble/setup.bash "
                           "and ~/kinect_ws/install/setup.bash first. If tf2_ros is missing, "
                           "install it with: sudo apt install ros-humble-tf2-ros-py") from error

    class Scanner(Node):
        def __init__(self):
            super().__init__("plant_fusion")
            self.engine = create_engine(args, output)
            self.latest = None
            self.sequence, self.processed = 0, 0
            self.last_stamp = None
            self.model_revision = -1
            self.model_bytes, self.model_width = b"", 0
            self.origin_sent = False
            self.subscription = self.create_subscription(PointCloud2, args.topic, self.receive,
                QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
            self.model_pub = self.create_publisher(PointCloud2, "/plant_fusion/model",
                QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.status_pub = self.create_publisher(String, "/plant_fusion/status", 1)
            self.camera_tf = TransformBroadcaster(self)
            self.origin_tf = StaticTransformBroadcaster(self)
            self.save_service = self.create_service(Trigger, "/plant_fusion/save", self.save_request)
            self.recovery_service = self.create_service(Trigger, "/plant_fusion/relocalize", self.recovery_request)
            self.timer = self.create_timer(1.0 / args.rate, self.process)
            self.get_logger().info(f"Waiting for {args.topic}; crop {self.engine.bounds}; output {output}")
            if self.engine.accepted:
                self.publish_origin()
                self.publish_model()
                self.get_logger().info(f"Loaded {self.engine.accepted} saved captures. Return to an accepted view and hold still for recovery.")
            else:
                self.get_logger().info("Centre the entire plant in the first view. Keep it still; move the camera slowly.")
            self.get_logger().info(f"Target rate {args.rate:g}/s is a maximum, not achieved throughput. Wait for FUSED after each small camera step.")

        def receive(self, message):
            self.sequence += 1
            self.latest = (self.sequence, message)

        def publish_model(self):
            if self.model_revision != self.engine.accepted:
                points, counts = self.engine.model.arrays()
                record = np.empty(len(points), dtype=[("x", "<f4"), ("y", "<f4"),
                                                     ("z", "<f4"), ("observations", "<f4")])
                for k, name in enumerate(("x", "y", "z")):
                    record[name] = points[:, k]
                record["observations"] = counts
                self.model_bytes, self.model_width = record.tobytes(), len(points)
                self.model_revision = self.engine.accepted
            message = PointCloud2()
            message.header.stamp = self.get_clock().now().to_msg()
            message.header.frame_id = "plant_map"
            message.height, message.width = 1, self.model_width
            message.fields = [PointField(name=name, offset=i * 4, datatype=PointField.FLOAT32, count=1)
                              for i, name in enumerate(("x", "y", "z", "observations"))]
            message.is_bigendian, message.is_dense = False, True
            message.point_step, message.row_step = 16, 16 * self.model_width
            message.data = self.model_bytes
            self.model_pub.publish(message)

        def publish_origin(self):
            # plant_map is the first accepted optical camera coordinate system.
            # Retain that origin for late RViz subscribers even if tracking stops.
            if not self.origin_sent and self.engine.accepted:
                origin = TransformStamped()
                origin.header.stamp = self.get_clock().now().to_msg()
                origin.header.frame_id = "plant_map"
                origin.child_frame_id = "plant_scan_origin"
                origin.transform.rotation.w = 1.0
                self.origin_tf.sendTransform(origin)
                self.origin_sent = True

        def publish_pose(self, capture_stamp):
            self.publish_origin()
            camera = TransformStamped()
            camera.header.stamp = (capture_stamp if capture_stamp.sec or capture_stamp.nanosec
                                   else self.get_clock().now().to_msg())
            camera.header.frame_id = "plant_map"
            # A dedicated child avoids taking ownership of the driver's TF frame.
            # Its axes are the incoming XYZ cloud's optical camera axes.
            camera.child_frame_id = "plant_fusion_camera"
            position = self.engine.pose[:3, 3]
            camera.transform.translation.x = float(position[0])
            camera.transform.translation.y = float(position[1])
            camera.transform.translation.z = float(position[2])
            q = rotation_quaternion(self.engine.pose[:3, :3])
            camera.transform.rotation.x = float(q[0])
            camera.transform.rotation.y = float(q[1])
            camera.transform.rotation.z = float(q[2])
            camera.transform.rotation.w = float(q[3])
            self.camera_tf.sendTransform(camera)

        def process(self):
            if self.latest is None or self.latest[0] == self.processed:
                if self.engine.accepted:
                    self.publish_model()
                return
            self.processed, msg = self.latest
            stamp = int(msg.header.stamp.sec) * 1000000000 + int(msg.header.stamp.nanosec)
            if stamp and self.last_stamp is not None and stamp <= self.last_stamp:
                if self.engine.accepted:
                    self.publish_model()
                return
            input_gap = ((stamp - self.last_stamp) / 1e9
                         if stamp and self.last_stamp is not None else None)
            if stamp:
                self.last_stamp = stamp
            try:
                process_started = time.perf_counter()
                fields = [{"name": f.name, "offset": f.offset, "datatype": f.datatype, "count": f.count}
                          for f in msg.fields]
                xyz = decode_xyz(msg.data, msg.width, msg.height, msg.point_step, msg.row_step,
                                 fields, msg.is_bigendian)
                status = self.engine.update(xyz, msg.header.frame_id, stamp or None)
                if status["state"] == "fused":
                    self.publish_pose(msg.header.stamp)
                    self.publish_model()
                    if self.engine.accepted == 1 or self.engine.accepted % 10 == 0:
                        self.engine.save()
                else:
                    if self.engine.accepted:
                        self.publish_model()
                status["processing_s"] = round(time.perf_counter() - process_started, 3)
                status["input_gap_s"] = input_gap
                if status["state"] == "fused":
                    recovered = " RECOVERED" if status.get("relocalized") else ""
                    self.get_logger().info(f"FUSED{recovered} capture {self.engine.accepted}: "
                                           f"{status['model_points']} model points; processing {status['processing_s']:.2f} s")
                else:
                    hint = (" Pause and return toward the last accepted view."
                            if status["state"] == "not_fused" and status.get("code") != "voxel_limit" else "")
                    self.get_logger().warning(f"{status['state'].upper()}: {status['reason']}.{hint} "
                                              f"Processing {status['processing_s']:.2f} s")
                self.status_pub.publish(String(data=json.dumps(status, allow_nan=False)))
            except (ValueError, RuntimeError, OSError) as error:
                self.get_logger().error(str(error))

        def save_request(self, request, response):
            del request
            try:
                response.message = str(self.engine.save())
                response.success = True
            except (ValueError, OSError) as error:
                response.message, response.success = str(error), False
            return response

        def recovery_request(self, request, response):
            del request
            try:
                self.engine.request_recovery()
                response.message = "Recovery requested. Return to a previously accepted view and hold still for two captures."
                response.success = True
            except ValueError as error:
                response.message, response.success = str(error), False
            return response

    rclpy.init(args=[])
    node = None
    try:
        node = Scanner()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            if node is not None and node.engine.accepted:
                print(f"Saved model: {node.engine.save()}", flush=True)
        finally:
            if node is not None:
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


def main(argv=None):
    command = parser()
    args = command.parse_args(argv)
    values = [args.rate, args.voxel, *(args.bounds or [])]
    if not all(math.isfinite(x) for x in values):
        command.error("Rate, voxel and crop values must be finite")
    if not 0.1 <= args.rate <= 10 or not 0.002 <= args.voxel <= 0.05:
        command.error("Use --rate 0.1 to 10 and --voxel 0.002 to 0.05 metres")
    if not 10000 <= args.max_voxels <= 2000000:
        command.error("Use --max-voxels between 10000 and 2000000")
    if args.bounds and any(args.bounds[i] >= args.bounds[i + 1] for i in (0, 2, 4)):
        command.error("Each crop minimum must be smaller than its maximum")
    if args.resume and (args.no_record or args.bounds is not None):
        command.error("Resume preserves the saved crop and raw captures; omit --bounds and --no-record")
    output = (args.output or Path.home() / ("plant_scan_" + datetime.now().strftime("%Y%m%d_%H%M%S_%f"))).expanduser().resolve()
    if output.exists():
        command.error(f"Output already exists: {output}; choose a new directory")
    try:
        if args.self_test:
            self_test()
            return 0
        native_preflight()
        if args.replay:
            engine = create_engine(args, output)
            for capture in args.replay:
                xyz, frame = read_capture(capture)
                print(json.dumps(engine.update(xyz, frame), allow_nan=False), flush=True)
            if not engine.accepted:
                raise ValueError("No replay capture was accepted")
            print(f"Saved model: {engine.save()}")
        else:
            run_ros(args, output)
    except (ValueError, RuntimeError, OSError, zipfile.BadZipFile) as error:
        print(f"Scanner stopped: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
