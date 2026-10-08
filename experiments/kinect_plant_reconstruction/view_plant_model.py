#!/usr/bin/env python3
"""Publish the supplied binary XYZ PLY to RViz. Requires ROS 2 and NumPy."""
import argparse
from pathlib import Path
import numpy as np


def read_xyz(path):
    with Path(path).open('rb') as stream:
        lines=[]
        for _ in range(100):
            line=stream.readline().decode('ascii').strip()
            lines.append(line)
            if line=='end_header':break
        else:raise ValueError('Invalid PLY header')
        if 'format binary_little_endian 1.0' not in lines:
            raise ValueError('This viewer expects the supplied binary XYZ PLY.')
        props=[line for line in lines if line.startswith('property ')]
        if props!=['property float x','property float y','property float z']:
            raise ValueError('Expected float XYZ only; use the supplied model PLY.')
        count=int(next(line.split()[-1] for line in lines if line.startswith('element vertex ')))
        data=stream.read()
        if len(data)!=count*12:raise ValueError('PLY data size does not match header')
        points=np.frombuffer(data,dtype='<f4').reshape(-1,3)
        return points[np.isfinite(points).all(axis=1)]


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('ply',type=Path)
    args=parser.parse_args()
    xyz=read_xyz(args.ply)
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from sensor_msgs.msg import PointCloud2, PointField
    rclpy.init(args=[])
    node=rclpy.create_node('saved_plant_model')
    pub=node.create_publisher(PointCloud2,'/plant_offline/model',QoSProfile(
        depth=1,reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL))
    msg=PointCloud2()
    msg.header.frame_id='plant_map'
    msg.height=1;msg.width=len(xyz)
    msg.fields=[PointField(name=name,offset=i*4,datatype=7,count=1)
                for i,name in enumerate(('x','y','z'))]
    msg.is_bigendian=False;msg.is_dense=True
    msg.point_step=12;msg.row_step=12*len(xyz)
    msg.data=xyz.astype('<f4').tobytes()
    def publish():
        msg.header.stamp=node.get_clock().now().to_msg()
        pub.publish(msg)
    timer=node.create_timer(1.0,publish)
    publish()
    print(f'Publishing {len(xyz):,} points. RViz Fixed Frame: plant_map; topic: /plant_offline/model')
    print('Keep this terminal running. Ctrl+C stops the viewer.')
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
