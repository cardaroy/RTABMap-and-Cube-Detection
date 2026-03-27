#!/usr/bin/env python3
"""
Save confirmed cube markers from /perception/cube_map_markers to a JSON file.

Run during a live SLAM session:
  python3 src/save_cube_markers.py markers.json

Press Ctrl+C to stop — it saves the latest snapshot.
"""

import sys
import json
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray, Marker


class MarkerSaver(Node):
    def __init__(self, out_path: str):
        super().__init__('marker_saver')
        self.out_path = out_path
        self.markers = []
        self.sub = self.create_subscription(
            MarkerArray, '/perception/cube_map_markers', self._cb, 10)
        self.get_logger().info('Listening on /perception/cube_map_markers … Ctrl+C to save and exit.')

    def _cb(self, msg: MarkerArray):
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
        if cubes:
            self.markers = cubes
            self.get_logger().info(f'{len(cubes)} cubes captured')

    def save(self):
        with open(self.out_path, 'w') as f:
            json.dump(self.markers, f, indent=2)
        self.get_logger().info(f'Saved {len(self.markers)} cubes to {self.out_path}')


def main():
    if len(sys.argv) < 2:
        print('Usage: python3 save_cube_markers.py <output.json>')
        sys.exit(1)

    rclpy.init()
    node = MarkerSaver(sys.argv[1])
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.save()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
