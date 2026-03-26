#!/usr/bin/env python3
"""
Listens to /perception/target_pose_cam (PoseStamped in camera frame),
transforms each detection into the 'map' frame via TF, and publishes
persistent markers so detected cubes stay fixed on the RTAB-Map.

Deduplication: if a new detection is within `merge_radius_m` of an
existing marker, it updates that marker instead of creating a duplicate.
"""
import math
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA

from tf2_ros import Buffer, TransformListener, TransformException
from tf2_geometry_msgs import do_transform_pose_stamped


class CubeMapMarkerNode(Node):
    def __init__(self):
        super().__init__("cube_map_marker_node")

        self.declare_parameter("target_frame", "map")
        self.declare_parameter("merge_radius_m", 0.15)
        self.declare_parameter("marker_scale", 0.06)
        self.declare_parameter("marker_lifetime_sec", 0.0)  # 0 = forever

        self.target_frame = str(self.get_parameter("target_frame").value)
        self.merge_radius = float(self.get_parameter("merge_radius_m").value)
        self.marker_scale = float(self.get_parameter("marker_scale").value)
        self.marker_lifetime = float(self.get_parameter("marker_lifetime_sec").value)

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Stored landmarks: list of (x, y, z)
        self._landmarks: List[Tuple[float, float, float]] = []

        # Sub / pub
        self.create_subscription(
            PoseStamped, "/perception/target_pose_cam", self.on_target, 10
        )
        self.pub_markers = self.create_publisher(
            MarkerArray, "/perception/cube_map_markers", 10
        )

        self.get_logger().info(
            f"CubeMapMarkerNode started — transforming detections to '{self.target_frame}' frame."
        )

    def on_target(self, msg: PoseStamped):
        # Transform pose into map frame
        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame, msg.header.frame_id,
                rclpy.time.Time(), timeout=Duration(seconds=0.1)
            )
        except TransformException as e:
            self.get_logger().warn(f"TF lookup failed: {e}", throttle_duration_sec=2.0)
            return

        pose_map = do_transform_pose_stamped(msg, transform)

        x = pose_map.pose.position.x
        y = pose_map.pose.position.y
        z = pose_map.pose.position.z

        # Merge with existing landmark if close enough
        merged = False
        for i, (lx, ly, lz) in enumerate(self._landmarks):
            dist = math.sqrt((x - lx) ** 2 + (y - ly) ** 2 + (z - lz) ** 2)
            if dist < self.merge_radius:
                # Running average to refine position
                self._landmarks[i] = (
                    (lx + x) / 2.0,
                    (ly + y) / 2.0,
                    (lz + z) / 2.0,
                )
                merged = True
                break

        if not merged:
            self._landmarks.append((x, y, z))

        self._publish_all()

    def _publish_all(self):
        ma = MarkerArray()
        now = self.get_clock().now().to_msg()

        for i, (x, y, z) in enumerate(self._landmarks):
            # Sphere marker
            m = Marker()
            m.header.stamp = now
            m.header.frame_id = self.target_frame
            m.ns = "cube_landmarks"
            m.id = i
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose.position.x = x
            m.pose.position.y = y
            m.pose.position.z = z
            m.pose.orientation.w = 1.0
            m.scale.x = self.marker_scale
            m.scale.y = self.marker_scale
            m.scale.z = self.marker_scale
            m.color = ColorRGBA(r=1.0, g=0.4, b=0.0, a=0.9)  # orange
            if self.marker_lifetime > 0:
                m.lifetime = Duration(seconds=self.marker_lifetime).to_msg()
            ma.markers.append(m)

            # Text label
            t = Marker()
            t.header = m.header
            t.ns = "cube_landmark_text"
            t.id = 1000 + i
            t.type = Marker.TEXT_VIEW_FACING
            t.action = Marker.ADD
            t.pose.position.x = x
            t.pose.position.y = y
            t.pose.position.z = z + 0.10
            t.pose.orientation.w = 1.0
            t.scale.z = 0.06
            t.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            t.text = f"Cube #{i + 1}\n({x:.2f}, {y:.2f}, {z:.2f})"
            if self.marker_lifetime > 0:
                t.lifetime = Duration(seconds=self.marker_lifetime).to_msg()
            ma.markers.append(t)

        self.pub_markers.publish(ma)


def main():
    rclpy.init()
    node = CubeMapMarkerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
