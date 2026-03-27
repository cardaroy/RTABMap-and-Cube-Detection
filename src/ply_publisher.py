#!/usr/bin/env python3
"""
Publish a .ply point cloud file as a sensor_msgs/PointCloud2 topic for RViz.

Usage:
  ros2 run --prefix 'python3' -- src/ply_publisher.py /path/to/cloud.ply
  # or simply:
  python3 src/ply_publisher.py /path/to/cloud.ply

  Then in RViz add a PointCloud2 display on topic /ply_cloud (frame: map).
"""

import sys
import struct
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from plyfile import PlyData


class PlyPublisher(Node):
    def __init__(self, ply_path: str):
        super().__init__('ply_publisher')
        self.pub = self.create_publisher(PointCloud2, '/ply_cloud', 1)

        self.get_logger().info(f'Loading {ply_path} ...')
        ply = PlyData.read(ply_path)
        verts = ply['vertex']

        x = np.array(verts['x'], dtype=np.float32)
        y = np.array(verts['y'], dtype=np.float32)
        z = np.array(verts['z'], dtype=np.float32)

        has_color = all(name in verts.data.dtype.names for name in ('red', 'green', 'blue'))

        if has_color:
            r = np.array(verts['red'], dtype=np.uint8)
            g = np.array(verts['green'], dtype=np.uint8)
            b = np.array(verts['blue'], dtype=np.uint8)
            # Pack RGB into a single float32 (RViz convention)
            rgb_uint32 = (r.astype(np.uint32) << 16) | (g.astype(np.uint32) << 8) | b.astype(np.uint32)
            rgb_float = np.frombuffer(rgb_uint32.tobytes(), dtype=np.float32)

        n_points = len(x)
        self.get_logger().info(f'Loaded {n_points} points (color={has_color})')

        # Build PointCloud2 message
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        point_step = 12

        if has_color:
            fields.append(PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1))
            point_step = 16

        # Interleave into byte buffer
        if has_color:
            points = np.column_stack((x, y, z, rgb_float))
        else:
            points = np.column_stack((x, y, z))

        self.msg = PointCloud2()
        self.msg.header = Header(frame_id='map')
        self.msg.height = 1
        self.msg.width = n_points
        self.msg.fields = fields
        self.msg.is_bigendian = False
        self.msg.point_step = point_step
        self.msg.row_step = point_step * n_points
        self.msg.data = points.astype(np.float32).tobytes()
        self.msg.is_dense = True

        # Publish at 1 Hz (latched-like behavior)
        self.timer = self.create_timer(1.0, self._publish)
        self.get_logger().info('Publishing on /ply_cloud at 1 Hz (frame: map)')

    def _publish(self):
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.msg)


def main():
    if len(sys.argv) < 2:
        print('Usage: python3 ply_publisher.py <path_to_ply_file>')
        print(f'Available: /home/cardaroy/cloudvisuals/cloud.ply')
        sys.exit(1)

    rclpy.init()
    node = PlyPublisher(sys.argv[1])
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
