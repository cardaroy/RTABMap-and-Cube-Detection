#!/usr/bin/env python3
"""
Load cube markers from a JSON file and publish as MarkerArray for RViz.

Usage:
  python3 src/load_cube_markers.py markers.json

Publishes on /perception/cube_map_markers at 1 Hz (same topic as live node).
"""

import sys
import json
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA


class MarkerLoader(Node):
    def __init__(self, json_path: str):
        super().__init__('marker_loader')
        self.pub = self.create_publisher(MarkerArray, '/perception/cube_map_markers', 1)

        with open(json_path, 'r') as f:
            self.cubes = json.load(f)

        self.get_logger().info(f'Loaded {len(self.cubes)} cubes from {json_path}')
        self.timer = self.create_timer(1.0, self._publish)

    def _publish(self):
        ma = MarkerArray()
        now = self.get_clock().now().to_msg()

        for i, c in enumerate(self.cubes):
            # Cube marker
            m = Marker()
            m.header.stamp = now
            m.header.frame_id = 'map'
            m.ns = 'cube_landmarks'
            m.id = c.get('id', i + 1)
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose.position.x = c['x']
            m.pose.position.y = c['y']
            m.pose.position.z = c['z']
            m.pose.orientation.w = 1.0
            scale = c.get('scale', 0.1)
            m.scale.x = scale
            m.scale.y = scale
            m.scale.z = scale
            m.color = ColorRGBA(
                r=c.get('r', 1.0),
                g=c.get('g', 0.4),
                b=c.get('b', 0.0),
                a=c.get('a', 0.9),
            )
            ma.markers.append(m)

            # Text label
            t = Marker()
            t.header = m.header
            t.ns = 'cube_landmark_text'
            t.id = 1000 + m.id
            t.type = Marker.TEXT_VIEW_FACING
            t.action = Marker.ADD
            t.pose.position.x = c['x']
            t.pose.position.y = c['y']
            t.pose.position.z = c['z'] + 0.10
            t.pose.orientation.w = 1.0
            t.scale.z = 0.06
            t.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            t.text = f"Cube #{m.id}\n({c['x']:.2f}, {c['y']:.2f}, {c['z']:.2f})"
            ma.markers.append(t)

        self.pub.publish(ma)


def main():
    if len(sys.argv) < 2:
        print('Usage: python3 load_cube_markers.py <markers.json>')
        sys.exit(1)

    rclpy.init()
    node = MarkerLoader(sys.argv[1])
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
