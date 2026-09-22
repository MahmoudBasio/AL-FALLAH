#!/usr/bin/env python3

import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, Header
from tf2_ros import Buffer, TransformException, TransformListener


class TreeGenerator(Node):
    """Synthetic foliage + user-defined target geometry for Phase 1."""

    def __init__(self):
        super().__init__('tree_generator')

        # Phase 1 frame. For a moving-base Phase 2 system, use a fixed world frame
        # such as odom/map for the tree instead of base_footprint.
        self.declare_parameter('frame_id', 'base_footprint')
        self.declare_parameter('tool_frame', 'arm_tool_link')

        self.declare_parameter('tree_center_x', 0.90)
        self.declare_parameter('tree_center_y', 0.00)
        self.declare_parameter('tree_center_z', 1.00)

        self.declare_parameter('tree_radius_x', 0.42)
        self.declare_parameter('tree_radius_y', 0.42)
        self.declare_parameter('tree_radius_z', 0.70)
        self.declare_parameter('point_count', 15000)

        self.declare_parameter('box_width', 0.55)
        self.declare_parameter('box_depth', 0.55)
        self.declare_parameter('box_height', 1.00)
        self.declare_parameter('cut_radius', 0.07)
        self.declare_parameter('seed', 4)

        self.frame_id = self.get_parameter('frame_id').value
        self.tool_frame = self.get_parameter('tool_frame').value
        self.cut_radius = float(self.get_parameter('cut_radius').value)

        self.keep_pub = self.create_publisher(PointCloud2, '/tree/keep', 10)
        self.cut_pub = self.create_publisher(PointCloud2, '/tree/cut', 10)
        self.box_pub = self.create_publisher(PointCloud2, '/tree/target_box', 10)

        self.cutting_enabled = False
        self.create_subscription(Bool, '/tree/cutting_enabled', self.cutting_cb, 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.keep_points = np.empty((0, 3), dtype=np.float32)
        self.cut_points = np.empty((0, 3), dtype=np.float32)
        self.box_points = np.empty((0, 3), dtype=np.float32)

        self.generate_tree()
        self.create_timer(0.1, self.update_tree)

        self.get_logger().info(
            f'Tree model ready in frame {self.frame_id}; cutter frame: {self.tool_frame}'
        )

    def cutting_cb(self, msg: Bool):
        self.cutting_enabled = bool(msg.data)
        state = 'ENABLED' if self.cutting_enabled else 'DISABLED'
        self.get_logger().info(f'CUTTING {state}')

    def generate_tree(self):
        center = np.array([
            float(self.get_parameter('tree_center_x').value),
            float(self.get_parameter('tree_center_y').value),
            float(self.get_parameter('tree_center_z').value),
        ], dtype=np.float32)

        radii = np.array([
            float(self.get_parameter('tree_radius_x').value),
            float(self.get_parameter('tree_radius_y').value),
            float(self.get_parameter('tree_radius_z').value),
        ], dtype=np.float32)

        n = int(self.get_parameter('point_count').value)
        seed = int(self.get_parameter('seed').value)
        rng = np.random.default_rng(seed)

        # Uniformly sample the volume of an ellipsoid.
        directions = rng.normal(size=(n, 3)).astype(np.float32)
        directions /= np.linalg.norm(directions, axis=1, keepdims=True) + 1e-9
        radius = np.cbrt(rng.random(n)).astype(np.float32)[:, None]
        points = center + directions * radius * radii

        width = float(self.get_parameter('box_width').value)
        depth = float(self.get_parameter('box_depth').value)
        height = float(self.get_parameter('box_height').value)

        half = np.array([width / 2.0, depth / 2.0, height / 2.0], dtype=np.float32)
        inside = np.all(np.abs(points - center) <= half, axis=1)

        self.keep_points = points[inside].copy()
        self.cut_points = points[~inside].copy()
        self.box_points = self.generate_box_surface(center, width, depth, height)

        self.get_logger().info(
            f'Generated {len(points)} foliage points: '
            f'{len(self.keep_points)} keep, {len(self.cut_points)} cut.'
        )

    @staticmethod
    def generate_box_surface(center, width, depth, height, samples=24):
        x0, y0, z0 = center
        xs = np.linspace(x0 - width / 2, x0 + width / 2, samples)
        ys = np.linspace(y0 - depth / 2, y0 + depth / 2, samples)
        zs = np.linspace(z0 - height / 2, z0 + height / 2, samples)

        pts = []

        # Front / back faces
        for x in (x0 - width / 2, x0 + width / 2):
            for y in ys:
                for z in zs:
                    pts.append((x, y, z))

        # Left / right faces
        for y in (y0 - depth / 2, y0 + depth / 2):
            for x in xs:
                for z in zs:
                    pts.append((x, y, z))

        # Top / bottom faces
        for z in (z0 - height / 2, z0 + height / 2):
            for x in xs:
                for y in ys:
                    pts.append((x, y, z))

        return np.asarray(pts, dtype=np.float32)

    def update_tree(self):
        if self.cutting_enabled:
            self.remove_touched_excess_points()

        stamp = self.get_clock().now().to_msg()
        self.keep_pub.publish(self.make_cloud(self.keep_points, stamp))
        self.cut_pub.publish(self.make_cloud(self.cut_points, stamp))
        self.box_pub.publish(self.make_cloud(self.box_points, stamp))

    def make_cloud(self, points, stamp):
        header = Header()
        header.stamp = stamp
        header.frame_id = self.frame_id
        return point_cloud2.create_cloud_xyz32(header, points.tolist())

    def remove_touched_excess_points(self):
        """Remove only excess foliage contacted by the cutter tool frame."""
        if len(self.cut_points) == 0:
            return

        try:
            tf = self.tf_buffer.lookup_transform(
                self.frame_id,
                self.tool_frame,
                rclpy.time.Time(),
            )
        except TransformException:
            return

        p = tf.transform.translation
        tool = np.array([p.x, p.y, p.z], dtype=np.float32)

        distances = np.linalg.norm(self.cut_points - tool, axis=1)
        keep_mask = distances > self.cut_radius
        removed = int(np.count_nonzero(~keep_mask))

        if removed:
            self.cut_points = self.cut_points[keep_mask]
            self.get_logger().info(
                f'Virtual cutter removed {removed} excess foliage points; '
                f'{len(self.cut_points)} remain.'
            )


def main(args=None):
    rclpy.init(args=args)
    node = TreeGenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
