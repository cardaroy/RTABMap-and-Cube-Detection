#!/usr/bin/env python3
"""
Auto-save RTAB-Map point cloud and cube markers at regular intervals.

Usage:
  python3 src/auto_save.py                     # saves every 60s to ~/my_new_ws/saves/
  python3 src/auto_save.py --interval 30       # every 30s
  python3 src/auto_save.py --dir /tmp/saves    # custom output directory

Saves:
  <dir>/cloud_<timestamp>.ply       — 3D point cloud from /rtabmap/cloud_map
  <dir>/markers_<timestamp>.json    — cube markers from /perception/cube_map_markers
  <dir>/cloud_latest.ply            — always points to the most recent cloud
  <dir>/markers_latest.json         — always points to the most recent markers
"""

import argparse
import json
import os
import struct
import sys
from datetime import datetime

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray


class AutoSaver(Node):
    def __init__(self, save_dir: str, interval: float):
        super().__init__('auto_saver')
        self.save_dir = save_dir
        os.makedirs(save_dir, exist_ok=True)

        self.latest_cloud = None
        self.latest_markers = None

        self.sub_cloud = self.create_subscription(
            PointCloud2, '/rtabmap/cloud_map', self._cloud_cb, 1)
        self.sub_markers = self.create_subscription(
            MarkerArray, '/perception/cube_map_markers', self._markers_cb, 10)

        self.timer = self.create_timer(interval, self._save)
        self.get_logger().info(
            f'Auto-saving every {interval}s to {save_dir}')

    def _cloud_cb(self, msg: PointCloud2):
        self.latest_cloud = msg

    def _markers_cb(self, msg: MarkerArray):
        self.latest_markers = msg

    def _save(self):
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        saved = []

        if self.latest_cloud is not None:
            ply_path = os.path.join(self.save_dir, f'cloud_{ts}.ply')
            self._save_ply(self.latest_cloud, ply_path)
            # Update latest symlink
            latest = os.path.join(self.save_dir, 'cloud_latest.ply')
            if os.path.islink(latest) or os.path.exists(latest):
                os.remove(latest)
            os.symlink(ply_path, latest)
            saved.append(f'cloud ({self._count_points(self.latest_cloud)} pts)')

        if self.latest_markers is not None:
            json_path = os.path.join(self.save_dir, f'markers_{ts}.json')
            n = self._save_markers(self.latest_markers, json_path)
            latest = os.path.join(self.save_dir, 'markers_latest.json')
            if os.path.islink(latest) or os.path.exists(latest):
                os.remove(latest)
            os.symlink(json_path, latest)
            saved.append(f'markers ({n} cubes)')

        if saved:
            self.get_logger().info(f'Saved: {", ".join(saved)}')
        else:
            self.get_logger().info('No data received yet, skipping save.')

    def _count_points(self, msg: PointCloud2) -> int:
        return msg.width * msg.height

    def _save_ply(self, msg: PointCloud2, path: str):
        """Convert PointCloud2 to PLY with XYZ + RGB."""
        # Parse fields
        field_map = {f.name: f for f in msg.fields}
        has_rgb = 'rgb' in field_map

        n = msg.width * msg.height
        data = np.frombuffer(msg.data, dtype=np.uint8)

        # Extract XYZ
        x_off = field_map['x'].offset
        y_off = field_map['y'].offset
        z_off = field_map['z'].offset
        step = msg.point_step

        xyz = np.zeros((n, 3), dtype=np.float32)
        for i, axis_off in enumerate([x_off, y_off, z_off]):
            xyz[:, i] = np.frombuffer(
                msg.data, dtype=np.float32,
                count=n, offset=0
            ) if False else np.array([
                struct.unpack_from('<f', msg.data, j * step + axis_off)[0]
                for j in range(n)
            ], dtype=np.float32)

        # Vectorized extraction
        raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(n, step)
        xyz = np.column_stack([
            raw[:, off:off+4].view(np.float32).reshape(-1)
            for off in [x_off, y_off, z_off]
        ])

        rgb = None
        if has_rgb:
            rgb_off = field_map['rgb'].offset
            rgb_bytes = raw[:, rgb_off:rgb_off+4]
            rgb = np.column_stack([
                rgb_bytes[:, 2],  # R
                rgb_bytes[:, 1],  # G
                rgb_bytes[:, 0],  # B
            ])

        # Filter out NaN/Inf
        valid = np.isfinite(xyz).all(axis=1)
        xyz = xyz[valid]
        if rgb is not None:
            rgb = rgb[valid]

        # Write PLY
        with open(path, 'wb') as f:
            header = (
                "ply\n"
                "format binary_little_endian 1.0\n"
                f"element vertex {len(xyz)}\n"
                "property float x\n"
                "property float y\n"
                "property float z\n"
            )
            if rgb is not None:
                header += (
                    "property uchar red\n"
                    "property uchar green\n"
                    "property uchar blue\n"
                )
            header += "end_header\n"
            f.write(header.encode('ascii'))

            if rgb is not None:
                for i in range(len(xyz)):
                    f.write(struct.pack('<fff', *xyz[i]))
                    f.write(struct.pack('<BBB', *rgb[i]))
            else:
                xyz.tofile(f)

    def _save_markers(self, msg: MarkerArray, path: str) -> int:
        cubes = []
        for m in msg.markers:
            if m.type == Marker.CUBE:
                cubes.append({
                    'id': m.id,
                    'x': m.pose.position.x,
                    'y': m.pose.position.y,
                    'z': m.pose.position.z,
                    'r': m.color.r,
                    'g': m.color.g,
                    'b': m.color.b,
                    'a': m.color.a,
                    'scale': m.scale.x,
                })
        with open(path, 'w') as f:
            json.dump(cubes, f, indent=2)
        return len(cubes)


def main():
    parser = argparse.ArgumentParser(description='Auto-save cloud + markers')
    parser.add_argument('--interval', type=float, default=60.0,
                        help='Save interval in seconds (default: 60)')
    parser.add_argument('--dir', type=str,
                        default=os.path.expanduser('~/my_new_ws/saves'),
                        help='Output directory')
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = AutoSaver(args.dir, args.interval)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.get_logger().info('Shutting down — doing final save.')
    node._save()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
