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
import math
from typing import List

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA

from tf2_ros import Buffer, TransformListener, TransformException
from tf2_geometry_msgs import do_transform_pose_stamped


class _Landmark:
    __slots__ = ('x', 'y', 'z', 'obs_count')

    def __init__(self, x: float, y: float, z: float):
        self.x = x
        self.y = y
        self.z = z
        self.obs_count = 1

    def merge(self, x: float, y: float, z: float):
        """Running average to refine position, increment observation count."""
        self.x = (self.x + x) / 2.0
        self.y = (self.y + y) / 2.0
        self.z = (self.z + z) / 2.0
        self.obs_count += 1

    def dist_to(self, x: float, y: float, z: float) -> float:
        return math.sqrt((self.x - x) ** 2 + (self.y - y) ** 2 + (self.z - z) ** 2)


class CubeMapMarkerNode(Node):
    def __init__(self):
        super().__init__("cube_map_marker_node")

        self.declare_parameter("target_frame", "map")
        self.declare_parameter("merge_radius_m", 2)
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

        # Sub / pub
        self.create_subscription(
            PoseStamped, "/perception/target_pose_cam", self.on_target, 10
        )
        self.pub_markers = self.create_publisher(
            MarkerArray, "/perception/cube_map_markers", 10
        )

        self.get_logger().info(
            f"CubeMapMarkerNode started — need {self.min_observations} observations "
            f"to confirm a cube in '{self.target_frame}' frame."
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
        for lm in self._landmarks:
            if lm.dist_to(x, y, z) < self.merge_radius:
                was_confirmed = lm.obs_count >= self.min_observations
                lm.merge(x, y, z)
                if not was_confirmed and lm.obs_count >= self.min_observations:
                    self.get_logger().info(
                        f"Cube CONFIRMED at ({lm.x:.2f}, {lm.y:.2f}, {lm.z:.2f}) "
                        f"after {lm.obs_count} observations"
                    )
                merged = True
                break

        if not merged:
            self._landmarks.append(_Landmark(x, y, z))

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
            m.color = ColorRGBA(r=1.0, g=0.4, b=0.0, a=0.9)  # orange
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
            t.text = f"Cube #{confirmed_idx}\n({lm.x:.2f}, {lm.y:.2f}, {lm.z:.2f})\n[seen {lm.obs_count}x]"
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
