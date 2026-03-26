#!/usr/bin/env python3
import json
import os
import time
from typing import Dict, List, Optional

import cv2
import numpy as np
from ultralytics import YOLO

import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Float32MultiArray
from geometry_msgs.msg import Pose
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge


class CubeDetectorSimpleNode(Node):
    def __init__(self):
        super().__init__("cube_detector_simple_node")

        # Params
        DEFAULT_MODEL_PATH = "/home/yao/models/best.pt"
        self.declare_parameter("model_path", DEFAULT_MODEL_PATH)
        self.declare_parameter("conf_thres", 0.6)
        self.declare_parameter("depth_min_m", 0.15)
        self.declare_parameter("depth_max_m", 5.0)
        self.declare_parameter("frame_id", "camera_color_optical_frame")
        self.declare_parameter("publish_rate_hz", 15.0)

        self.declare_parameter("color_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("depth_topic", "/camera/camera/depth/image_rect_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera/color/camera_info")
        self.declare_parameter("depth_unit", "mm")   # "mm" or "m"
        self.declare_parameter("depth_roi_px", 9)    # odd number recommended

        # Read params
        self.model_path = str(self.get_parameter("model_path").value)
        self.conf_thres = float(self.get_parameter("conf_thres").value)
        self.depth_min_m = float(self.get_parameter("depth_min_m").value)
        self.depth_max_m = float(self.get_parameter("depth_max_m").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)

        self.color_topic = str(self.get_parameter("color_topic").value)
        self.depth_topic = str(self.get_parameter("depth_topic").value)
        self.camera_info_topic = str(self.get_parameter("camera_info_topic").value)

        self.depth_unit = str(self.get_parameter("depth_unit").value).lower()
        self.depth_roi_px = int(self.get_parameter("depth_roi_px").value)
        if self.depth_roi_px < 1:
            self.depth_roi_px = 1
        if self.depth_roi_px % 2 == 0:
            self.depth_roi_px += 1

        if not self.model_path or not os.path.isfile(self.model_path):
            raise RuntimeError(f"Model not found: {self.model_path}")

        # YOLO
        self.get_logger().info(f"Loading model: {self.model_path}")
        self.model = YOLO(self.model_path)

        # Buffers / intrinsics
        self.bridge = CvBridge()
        self._color_bgr: Optional[np.ndarray] = None
        self._depth_raw: Optional[np.ndarray] = None
        self.fx = self.fy = self.cx = self.cy = None

        # Subscribers
        self.create_subscription(Image, self.color_topic, self.on_color, 10)
        self.create_subscription(Image, self.depth_topic, self.on_depth, 10)
        self.create_subscription(CameraInfo, self.camera_info_topic, self.on_info, 10)

        # Publishers
        self.pub_simple = self.create_publisher(Float32MultiArray, "/perception/cube_simple", 10)
        self.pub_json = self.create_publisher(String, "/perception/cube_detections_json", 10)
        self.pub_markers = self.create_publisher(MarkerArray, "/perception/cube_markers", 10)

        # Timer
        period = 1.0 / max(1e-3, self.publish_rate_hz)
        self.create_timer(period, self.tick)

        self.get_logger().info("CubeDetectorSimpleNode started (perception-only).")

    def on_color(self, msg: Image):
        try:
            self._color_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            if img.ndim == 3 and img.shape[2] == 3:
                self._color_bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            else:
                self._color_bgr = img

    def on_depth(self, msg: Image):
        self._depth_raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

    def on_info(self, msg: CameraInfo):
        self.fx = float(msg.k[0])
        self.fy = float(msg.k[4])
        self.cx = float(msg.k[2])
        self.cy = float(msg.k[5])

    def _depth_to_m(self, depth_raw: np.ndarray) -> np.ndarray:
        d = depth_raw.astype(np.float32)
        if self.depth_unit == "mm":
            d *= 0.001
        d[~np.isfinite(d)] = np.nan
        return d

    def _median_depth_roi(self, depth_m: np.ndarray, u: int, v: int) -> Optional[float]:
        h, w = depth_m.shape[:2]
        half = self.depth_roi_px // 2
        xs, xe = max(0, u - half), min(w, u + half + 1)
        ys, ye = max(0, v - half), min(h, v + half + 1)
        roi = depth_m[ys:ye, xs:xe]
        if roi.size == 0:
            return None
        valid = roi[(roi > self.depth_min_m) & (roi < self.depth_max_m) & np.isfinite(roi)]
        if valid.size == 0:
            return None
        return float(np.median(valid))

    def tick(self):
        if self._color_bgr is None or self._depth_raw is None:
            return
        if self.fx is None or self.fy is None or self.cx is None or self.cy is None:
            return

        depth_m = self._depth_to_m(self._depth_raw)
        results = self.model(self._color_bgr, conf=self.conf_thres, verbose=False)
        r0 = results[0]

        detections: List[Dict] = []
        markers = MarkerArray()
        flat: List[float] = []

        if r0.boxes is not None and len(r0.boxes) > 0:
            h, w = depth_m.shape[:2]

            for i, box in enumerate(r0.boxes):
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int).tolist()
                conf = float(box.conf[0].cpu().numpy())
                cls_id = int(box.cls[0].cpu().numpy())
                cls_name = self.model.names.get(cls_id, str(cls_id))

                # clamp bbox
                x1 = max(0, min(x1, w - 1))
                x2 = max(0, min(x2, w - 1))
                y1 = max(0, min(y1, h - 1))
                y2 = max(0, min(y2, h - 1))
                if x2 <= x1 or y2 <= y1:
                    continue

                u = int((x1 + x2) / 2)
                v = int((y1 + y2) / 2)

                Z = self._median_depth_roi(depth_m, u, v)
                if Z is None:
                    continue

                # Fill simple array: [x1,y1,x2,y2,conf,depth]
                flat.extend([float(x1), float(y1), float(x2), float(y2), float(conf), float(Z)])

                # Also publish JSON for human/debug
                detections.append({
                    "class": cls_name,
                    "conf": round(conf, 4),
                    "bbox_xyxy": [int(x1), int(y1), int(x2), int(y2)],
                    "uv": [u, v],
                    "depth_m": round(Z, 4),
                })

                # Markers (a point in front of camera along Z; not true X/Y without intrinsics)
                # For simple mode, marker is optional. We'll place at (0,0,Z) for visibility.
                p = Pose()
                p.position.x = 0.0
                p.position.y = 0.0
                p.position.z = float(Z)
                p.orientation.w = 1.0

                m = Marker()
                m.header.stamp = self.get_clock().now().to_msg()
                m.header.frame_id = self.frame_id
                m.ns = "cubes_simple"
                m.id = i
                m.type = Marker.SPHERE
                m.action = Marker.ADD
                m.pose = p
                m.scale.x = 0.06
                m.scale.y = 0.06
                m.scale.z = 0.06
                m.color.a = 1.0
                markers.markers.append(m)

        # Publish simple array
        arr = Float32MultiArray()
        arr.data = flat
        self.pub_simple.publish(arr)

        # Publish JSON
        js = String()
        js.data = json.dumps(
            {"stamp": time.time(), "frame_id": self.frame_id, "detections": detections},
            ensure_ascii=False,
        )
        self.pub_json.publish(js)

        # Publish markers
        self.pub_markers.publish(markers)


def main():
    rclpy.init()
    node = CubeDetectorSimpleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()