#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from visualization_msgs.msg import Marker


class PathGenerator(Node):
    """Publishes a Phase 1 raster trimming path on the front target face."""

    def __init__(self):
        super().__init__('path_generator')

        self.declare_parameter('frame_id', 'base_footprint')
        self.declare_parameter('tree_center_x', 0.90)
        self.declare_parameter('tree_center_y', 0.00)
        self.declare_parameter('tree_center_z', 1.00)
        self.declare_parameter('box_width', 0.55)
        self.declare_parameter('box_depth', 0.55)
        self.declare_parameter('box_height', 1.00)
        self.declare_parameter('rows', 8)

        self.pub = self.create_publisher(Marker, '/tree/trimming_path', 10)
        self.create_timer(0.5, self.publish_path)

    def publish_path(self):
        frame = self.get_parameter('frame_id').value
        cx = float(self.get_parameter('tree_center_x').value)
        cy = float(self.get_parameter('tree_center_y').value)
        cz = float(self.get_parameter('tree_center_z').value)
        width = float(self.get_parameter('box_width').value)
        depth = float(self.get_parameter('box_depth').value)
        height = float(self.get_parameter('box_height').value)
        rows = max(2, int(self.get_parameter('rows').value))

        # Front target face in the current Phase 1 convention.
        x = cx - width / 2.0
        y_min = cy - depth / 2.0
        y_max = cy + depth / 2.0
        z_min = cz - height / 2.0
        z_max = cz + height / 2.0

        marker = Marker()
        marker.header.frame_id = frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'trimming_path'
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.015
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        for i in range(rows):
            z = z_min + (z_max - z_min) * i / (rows - 1)
            if i % 2 == 0:
                y0, y1 = y_min, y_max
            else:
                y0, y1 = y_max, y_min

            p0 = Point(x=x, y=y0, z=z)
            p1 = Point(x=x, y=y1, z=z)
            marker.points.extend([p0, p1])

        self.pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = PathGenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
