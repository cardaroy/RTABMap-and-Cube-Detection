#!/usr/bin/env python3
"""
Listens to /perception/target_pose_cam (PoseStamped in camera frame),
transforms each detection into the 'map' frame via TF, and publishes
persistent markers so detected cubes stay fixed on the RTAB-Map.

Deduplication: if a new detection is within `merge_radius_m` of an
existing marker, it updates that marker instead of creating a duplicate.

Confirmation: a landmark must be observed `min_observations` times before
it is published as a confirmed marker. This filters out false positives.
"""
import json
import math
from typing import List, Optional

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA, String

from tf2_ros import Buffer, TransformListener, TransformException
from tf2_geometry_msgs import do_transform_pose_stamped

# Map color names to RGBA for cube markers
_COLOR_MAP = {
    "red":    ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.9),
    "orange": ColorRGBA(r=1.0, g=0.5, b=0.0, a=0.9),
    "yellow": ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.9),
    "green":  ColorRGBA(r=0.0, g=0.8, b=0.0, a=0.9),
    "blue":   ColorRGBA(r=0.0, g=0.3, b=1.0, a=0.9),
    "purple": ColorRGBA(r=0.6, g=0.0, b=0.8, a=0.9),
    "white":  ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.9),
    "gray":   ColorRGBA(r=0.6, g=0.6, b=0.6, a=0.9),
}
_DEFAULT_COLOR = ColorRGBA(r=1.0, g=0.4, b=0.0, a=0.9)


class _Landmark:
    __slots__ = ('x', 'y', 'z', 'obs_count', 'color_name')

    def __init__(self, x: float, y: float, z: float, color_name: str = "unknown"):
        self.x = x
        self.y = y
        self.z = z
        self.obs_count = 1
        self.color_name = color_name

    def merge(self, x: float, y: float, z: float, color_name: str = "unknown"):
        """Running average to refine position, increment observation count."""
        self.x = (self.x + x) / 2.0
        self.y = (self.y + y) / 2.0
        self.z = (self.z + z) / 2.0
        self.obs_count += 1
        if color_name != "unknown":
            self.color_name = color_name

    def dist_to(self, x: float, y: float, z: float) -> float:
        return math.sqrt((self.x - x) ** 2 + (self.y - y) ** 2 + (self.z - z) ** 2)


class CubeMapMarkerNode(Node):
    def __init__(self):
        super().__init__("cube_map_marker_node")

        self.declare_parameter("target_frame", "map")
        self.declare_parameter("merge_radius_m", 1.0)
        self.declare_parameter("min_observations", 5)
        self.declare_parameter("marker_scale", 0.06)
        self.declare_parameter("marker_lifetime_sec", 0.0)  # 0 = forever

        self.target_frame = str(self.get_parameter("target_frame").value)
        self.merge_radius = float(self.get_parameter("merge_radius_m").value)
        self.min_observations = int(self.get_parameter("min_observations").value)
        self.marker_scale = float(self.get_parameter("marker_scale").value)
        self.marker_lifetime = float(self.get_parameter("marker_lifetime_sec").value)

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Stored landmarks (confirmed + candidates)
        self._landmarks: List[_Landmark] = []
        self._latest_color: str = "unknown"  # color from most recent JSON

        # Sub / pub
        self.create_subscription(
            PoseStamped, "/perception/target_pose_cam", self.on_target, 10
        )
        self.create_subscription(
            String, "/perception/cube_detections_json", self._on_json, 10
        )
        self.pub_markers = self.create_publisher(
            MarkerArray, "/perception/cube_map_markers", 10
        )

        self.get_logger().info(
            f"CubeMapMarkerNode started — need {self.min_observations} observations "
            f"to confirm a cube in '{self.target_frame}' frame."
        )

    def _on_json(self, msg: String):
        """Extract best detection color from JSON."""
        try:
            data = json.loads(msg.data)
            best = data.get("best")
            detections = data.get("detections", [])
            # Use best detection's color, or first detection with a color
            if detections:
                for d in detections:
                    c = d.get("color", "unknown")
                    if c != "unknown":
                        self._latest_color = c
                        return
        except (json.JSONDecodeError, TypeError):
            pass

    def on_target(self, msg: PoseStamped):
        # Grab color from latest JSON detection
        color_name = self._latest_color

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
        for lm in self._landmarks:
            if lm.dist_to(x, y, z) < self.merge_radius:
                was_confirmed = lm.obs_count >= self.min_observations
                lm.merge(x, y, z, color_name)
                if not was_confirmed and lm.obs_count >= self.min_observations:
                    self.get_logger().info(
                        f"Cube CONFIRMED at ({lm.x:.2f}, {lm.y:.2f}, {lm.z:.2f}) "
                        f"color={lm.color_name} after {lm.obs_count} observations"
                    )
                merged = True
                break

        if not merged:
            self._landmarks.append(_Landmark(x, y, z, color_name))

        self._publish_all()

    def _publish_all(self):
        ma = MarkerArray()
        now = self.get_clock().now().to_msg()

        confirmed_idx = 0
        for lm in self._landmarks:
            if lm.obs_count < self.min_observations:
                continue  # not enough observations yet

            confirmed_idx += 1

            # Cube marker
            m = Marker()
            m.header.stamp = now
            m.header.frame_id = self.target_frame
            m.ns = "cube_landmarks"
            m.id = confirmed_idx
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose.position.x = lm.x
            m.pose.position.y = lm.y
            m.pose.position.z = lm.z
            m.pose.orientation.w = 1.0
            m.scale.x = self.marker_scale
            m.scale.y = self.marker_scale
            m.scale.z = self.marker_scale
            m.color = _COLOR_MAP.get(lm.color_name, _DEFAULT_COLOR)
            if self.marker_lifetime > 0:
                m.lifetime = Duration(seconds=self.marker_lifetime).to_msg()
            ma.markers.append(m)

            # Text label
            t = Marker()
            t.header = m.header
            t.ns = "cube_landmark_text"
            t.id = 1000 + confirmed_idx
            t.type = Marker.TEXT_VIEW_FACING
            t.action = Marker.ADD
            t.pose.position.x = lm.x
            t.pose.position.y = lm.y
            t.pose.position.z = lm.z + 0.10
            t.pose.orientation.w = 1.0
            t.scale.z = 0.06
            t.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            t.text = f"Cube #{confirmed_idx} [{lm.color_name}]\n({lm.x:.2f}, {lm.y:.2f}, {lm.z:.2f})\n[seen {lm.obs_count}x]"
            if self.marker_lifetime > 0:
                t.lifetime = Duration(seconds=self.marker_lifetime).to_msg()
            ma.markers.append(t)

        # Origin marker — rover start position
        origin = Marker()
        origin.header.stamp = now
        origin.header.frame_id = self.target_frame
        origin.ns = "origin"
        origin.id = 0
        origin.type = Marker.ARROW
        origin.action = Marker.ADD
        origin.pose.position.x = 0.0
        origin.pose.position.y = 0.0
        origin.pose.position.z = 0.0
        origin.pose.orientation.w = 1.0
        origin.scale.x = 0.15  # arrow length
        origin.scale.y = 0.03  # arrow width
        origin.scale.z = 0.03  # arrow height
        origin.color = ColorRGBA(r=0.0, g=1.0, b=1.0, a=1.0)  # cyan
        ma.markers.append(origin)

        origin_text = Marker()
        origin_text.header = origin.header
        origin_text.ns = "origin_text"
        origin_text.id = 0
        origin_text.type = Marker.TEXT_VIEW_FACING
        origin_text.action = Marker.ADD
        origin_text.pose.position.z = 0.15
        origin_text.pose.orientation.w = 1.0
        origin_text.scale.z = 0.08
        origin_text.color = ColorRGBA(r=0.0, g=1.0, b=1.0, a=1.0)
        origin_text.text = "START"
        ma.markers.append(origin_text)

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
