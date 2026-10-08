#!/usr/bin/env python3
"""Enter-triggered ROS 2 XYZ captures. Saves separate views, WITHOUT estimated poses.

Requires ROS 2, sensor_msgs and NumPy; does not import Open3D or SciPy.
Run the Kinect driver separately, using the same ROS environment.
RViz preview: /plant_capture/latest, Fixed Frame = input cloud frame (kinect_depth).
Each successful capture REPLACES the preview. Views are not aligned or merged.
"""
import argparse
import json
import threading
import time
import warnings
from datetime import datetime
from pathlib import Path

import numpy as np


def decode_xyz(msg):
    """Read organized PointCloud2, including row padding and byte order."""
    if msg.height <= 1 or msg.width <= 0:
        raise ValueError("Expected an organized depth cloud (height > 1).")
    fields = {f.name: f for f in msg.fields}
    endian = ">" if msg.is_bigendian else "<"
    formats, offsets = [], []
    for name in ("x", "y", "z"):
        field = fields.get(name)
        if field is None or field.datatype not in (7, 8) or field.count != 1:
            raise ValueError("XYZ fields must be scalar FLOAT32 or FLOAT64.")
        formats.append(endian + ("f4" if field.datatype == 7 else "f8"))
        offsets.append(field.offset)
    if msg.row_step < msg.width * msg.point_step:
        raise ValueError("Invalid PointCloud2 row stride.")
    if len(msg.data) < msg.height * msg.row_step:
        raise ValueError("Truncated PointCloud2 data.")
    dtype = np.dtype(dict(names=["x", "y", "z"], formats=formats,
                         offsets=offsets, itemsize=msg.point_step))
    points = np.ndarray((msg.height, msg.width), dtype=dtype,
                        buffer=msg.data, strides=(msg.row_step, msg.point_step))
    return np.stack([points[n] for n in ("x", "y", "z")], axis=-1).astype(np.float32)


class Inbox:
    def __init__(self):
        self.condition = threading.Condition()
        self.sequence = 0
        self.message = None
        self.error = None

    def push(self, msg):
        with self.condition:
            self.sequence += 1
            self.message = msg
            self.condition.notify_all()

    def mark(self):
        with self.condition:
            return self.sequence

    def next(self, after, timeout):
        deadline = time.monotonic() + timeout
        with self.condition:
            while self.sequence <= after:
                if self.error:
                    raise RuntimeError(self.error)
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError("No fresh /points data. Check driver and matching ROS settings.")
                self.condition.wait(min(remaining, 0.1))
            return self.sequence, self.message


def capture(inbox, count, interval, timeout):
    sequence = inbox.mark()  # Never capture the frame left over while repositioning.
    frames, stamps = [], []
    identity = None
    deadline = time.monotonic() + timeout
    while len(frames) < count:
        sequence, msg = inbox.next(sequence, max(0, deadline - time.monotonic()))
        if time.monotonic() >= deadline:
            raise TimeoutError("Timed out collecting distinct frames; view was not saved.")
        stamp = int(msg.header.stamp.sec) * 1000000000 + int(msg.header.stamp.nanosec)
        if stamps and stamp and stamp <= stamps[-1]:
            continue
        current = (msg.header.frame_id, msg.height, msg.width)
        if not current[0]:
            raise ValueError("Cloud has an empty frame_id.")
        if identity is not None and current != identity:
            raise ValueError("Cloud frame or dimensions changed during capture; retry.")
        identity = current
        frames.append(decode_xyz(msg))
        stamps.append(stamp)
        deadline = time.monotonic() + timeout
        if len(frames) < count:
            time.sleep(interval)
            sequence = inbox.mark()
    return np.stack(frames), np.array(stamps, dtype=np.int64), identity[0]


def median_view(frames):
    valid = np.isfinite(frames).all(axis=-1) & (frames[..., 2] > 0)
    clean = np.where(valid[..., None], frames, np.nan)
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", RuntimeWarning)
        median = np.nanmedian(clean, axis=0)
    keep = valid.sum(axis=0) >= int(np.ceil(len(frames) * 0.6))
    median[~keep] = np.nan
    return median.astype(np.float32)


def preview_message(xyz, frame_id, stamp, cloud_type, field_type):
    xyz = np.asarray(xyz, dtype=np.float32).reshape(-1, 3)
    xyz = xyz[np.isfinite(xyz).all(axis=1)]
    msg = cloud_type()
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp
    msg.height = 1
    msg.width = len(xyz)
    msg.fields = [field_type(name=name, offset=i * 4,
                             datatype=7, count=1)
                  for i, name in enumerate(("x", "y", "z"))]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = msg.width * 12
    msg.is_dense = True
    msg.data = xyz.astype("<f4").tobytes()
    return msg


def save_view(directory, number, frames, stamps, frame_id):
    organized = median_view(frames)
    xyz = organized[np.isfinite(organized).all(axis=-1)]
    if not len(xyz):
        raise ValueError("No valid depth points; view was not saved.")
    base = directory / f"view_{number:06d}"
    npz = base.with_suffix(".npz")
    ply = base.with_suffix(".ply")
    if npz.exists() or ply.exists():
        raise FileExistsError(f"Refusing to overwrite {base}")
    temporary = base.with_suffix(".npz.tmp")
    with temporary.open("wb") as stream:
        np.savez_compressed(stream, xyz=xyz, xyz_organized=organized,
                            xyz_frames=frames, stamp_ns=stamps,
                            frame_id=np.array(frame_id),
                            pose_known=np.array(False))
    temporary.replace(npz)
    temporary = base.with_suffix(".ply.tmp")
    with temporary.open("wb") as stream:
        stream.write(("ply\nformat binary_little_endian 1.0\n"
                      f"element vertex {len(xyz)}\nproperty float x\n"
                      "property float y\nproperty float z\nend_header\n").encode())
        stream.write(xyz.astype("<f4").tobytes())
    temporary.replace(ply)
    return dict(view=number, npz=npz.name, ply=ply.name,
                points=len(xyz), frame_id=frame_id, frames=len(frames),
                stamp_ns=stamps.tolist(), pose_known=False)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic", default="/points")
    parser.add_argument("--frames", type=int, default=10)
    parser.add_argument("--settle", type=float, default=2.0,
                        help="Seconds to hold still after pressing Enter (default: 2)")
    parser.add_argument("--interval", type=float, default=0.1)
    parser.add_argument("--timeout", type=float, default=15)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if args.frames < 1 or args.settle < 0 or args.interval < 0 or args.timeout <= 0:
        parser.error("frames/timeout must be positive; settle/interval must be nonnegative")
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import (qos_profile_sensor_data, QoSProfile,
                          ReliabilityPolicy, DurabilityPolicy)
    from sensor_msgs.msg import PointCloud2, PointField

    output = (args.output or Path.home() / datetime.now().strftime("plant_views_%Y%m%d_%H%M%S_%f")).expanduser()
    output.mkdir(parents=True, exist_ok=False)
    manifest = dict(format="kinect_manual_views_v1", topic=args.topic,
                    units="metres", poses_known=False, views=[])

    def save_manifest():
        temporary = output / "session.json.tmp"
        temporary.write_text(json.dumps(manifest, indent=2) + "\n")
        temporary.replace(output / "session.json")

    save_manifest()
    rclpy.init(args=[])
    node = rclpy.create_node("plant_capture_manual")
    inbox = Inbox()
    subscription = node.create_subscription(PointCloud2, args.topic, inbox.push, qos_profile_sensor_data)
    preview_pub = node.create_publisher(
        PointCloud2, "/plant_capture/latest",
        QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                   durability=DurabilityPolicy.TRANSIENT_LOCAL))
    preview_lock = threading.Lock()
    preview = None

    def publish_preview():
        # Repeat the LAST SAVED view so RViz opened later also receives it.
        # Never stream repositioning frames onto this topic.
        with preview_lock:
            if preview is not None:
                preview.header.stamp = node.get_clock().now().to_msg()
                preview_pub.publish(preview)

    preview_timer = node.create_timer(1.0, publish_preview)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    stop = threading.Event()

    def spin():
        try:
            while not stop.is_set() and rclpy.ok():
                executor.spin_once(timeout_sec=0.1)
        except Exception as exc:
            with inbox.condition:
                inbox.error = str(exc)
                inbox.condition.notify_all()

    worker = threading.Thread(target=spin, daemon=True)
    worker.start()
    print(f"Saving separate camera views to {output}")
    print("Keep the plant still. Reposition only BETWEEN captures. No automatic fusion.")
    print("RViz topic: /plant_capture/latest. Each saved view replaces the preview.")
    try:
        while rclpy.ok():
            command = input("\nEnter: capture next view | q + Enter: finish > ").strip().lower()
            if command == "q":
                break
            if command:
                print("Press Enter with no text to capture, or type q to finish.")
                continue
            print(f"Hold the camera still. Settling for {args.settle:g} seconds...", flush=True)
            time.sleep(args.settle)
            print(f"Recording {args.frames} fresh frames—keep still...", flush=True)
            try:
                frames, stamps, frame_id = capture(inbox, args.frames, args.interval, args.timeout)
                record = save_view(output, len(manifest["views"]) + 1, frames, stamps, frame_id)
            except (TimeoutError, ValueError) as exc:
                print(f"NOT SAVED: {exc}")
                continue
            manifest["views"].append(record)
            save_manifest()
            new_preview = preview_message(median_view(frames), frame_id,
                                          node.get_clock().now().to_msg(),
                                          PointCloud2, PointField)
            with preview_lock:
                preview = new_preview
            publish_preview()
            print(f"SAVED view {record['view']}: {record['points']} points. You may move now.", flush=True)
            print(f"RViz Fixed Frame: {frame_id}; topic: /plant_capture/latest")
    except (KeyboardInterrupt, EOFError):
        print("\nStopping. Previously saved views are retained.")
    finally:
        stop.set()
        worker.join(timeout=2)
        executor.shutdown(timeout_sec=2)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print(f"Saved {len(manifest['views'])} views: {output}")


if __name__ == "__main__":
    main()
