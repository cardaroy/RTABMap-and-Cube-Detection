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

from std_msgs.msg import String
from geometry_msgs.msg import Pose, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge


class CubeDetectorStandardNode(Node):
    def __init__(self):
        super().__init__("cube_detector_standard_node")

        # Params
        DEFAULT_MODEL_PATH = "/home/cardaroy/my_new_ws/Yao_Node_2/yolomodel_python/best.pt"
        self.declare_parameter("model_path", DEFAULT_MODEL_PATH)
        self.declare_parameter("conf_thres", 0.6)
        self.declare_parameter("depth_min_m", 0.15)
        self.declare_parameter("depth_max_m", 5.0)
        self.declare_parameter("frame_id", "camera_color_optical_frame")
        self.declare_parameter("publish_rate_hz", 15.0)

        self.declare_parameter("color_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("depth_topic", "/camera/camera/depth/image_rect_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera/color/camera_info")
        self.declare_parameter("depth_unit", "mm")
        self.declare_parameter("depth_roi_px", 9)

        self.declare_parameter("publish_json", True)
        self.declare_parameter("show_debug", True)

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

        self.publish_json = bool(self.get_parameter("publish_json").value)
        self.show_debug = bool(self.get_parameter("show_debug").value)

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
        self.pub_target_pose = self.create_publisher(PoseStamped, "/perception/target_pose_cam", 10)
        self.pub_markers = self.create_publisher(MarkerArray, "/perception/cube_markers", 10)
        self.pub_json = self.create_publisher(String, "/perception/cube_detections_json", 10)

        # Timer
        period = 1.0 / max(1e-3, self.publish_rate_hz)
        self.create_timer(period, self.tick)

        self.get_logger().info("CubeDetectorStandardNode started (perception-only).")

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

    def _classify_color(self, bgr_img: np.ndarray, x1: int, y1: int, x2: int, y2: int) -> str:
        """Sample center 40% of bbox and classify dominant color via HSV."""
        h, w = bgr_img.shape[:2]
        cx1 = x1 + int((x2 - x1) * 0.3)
        cx2 = x1 + int((x2 - x1) * 0.7)
        cy1 = y1 + int((y2 - y1) * 0.3)
        cy2 = y1 + int((y2 - y1) * 0.7)
        cx1, cx2 = max(0, cx1), min(w, cx2)
        cy1, cy2 = max(0, cy1), min(h, cy2)
        roi = bgr_img[cy1:cy2, cx1:cx2]
        if roi.size == 0:
            return "unknown"
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mean_h = float(np.mean(hsv[:, :, 0]))
        mean_s = float(np.mean(hsv[:, :, 1]))
        mean_v = float(np.mean(hsv[:, :, 2]))
        if mean_v < 40:
            return "black"
        if mean_s < 30 and mean_v > 200:
            return "white"
        if mean_s < 40:
            return "gray"
        if mean_h < 10 or mean_h > 170:
            return "red"
        if mean_h < 25:
            return "orange"
        if mean_h < 35:
            return "yellow"
        if mean_h < 85:
            return "green"
        if mean_h < 130:
            return "blue"
        if mean_h < 170:
            return "purple"
        return "unknown"

    def tick(self):
        if self._color_bgr is None or self._depth_raw is None:
            return
        if self.fx is None or self.fy is None or self.cx is None or self.cy is None:
            return

        depth_m = self._depth_to_m(self._depth_raw)
        debug_frame = self._color_bgr.copy() if self.show_debug else None
        results = self.model(self._color_bgr, conf=self.conf_thres, verbose=False)
        r0 = results[0]

        detections: List[Dict] = []
        markers = MarkerArray()

        best_xyz = None  # (X,Y,Z)
        best_z = 1e9
        best_conf = 0.0
        best_cls = "cube"

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

                X = (u - self.cx) * Z / self.fx
                Y = (v - self.cy) * Z / self.fy

                detections.append({
                    "class": cls_name,
                    "conf": round(conf, 4),
                    "bbox_xyxy": [int(x1), int(y1), int(x2), int(y2)],
                    "uv": [u, v],
                    "xyz_m": [round(float(X), 4), round(float(Y), 4), round(float(Z), 4)],
                })

                # Draw on debug frame
                if debug_frame is not None:
                    color_name = self._classify_color(self._color_bgr, x1, y1, x2, y2)
                    cv2.rectangle(debug_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    label = f"{cls_name} {conf:.2f} Z={Z:.2f}m [{color_name}]"
                    (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 2)
                    cv2.rectangle(debug_frame, (x1, y1 - th - 6), (x1 + tw, y1), (0, 255, 0), -1)
                    cv2.putText(debug_frame, label, (x1, y1 - 4),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 2)

                # choose nearest target
                if Z < best_z:
                    best_z = Z
                    best_xyz = (X, Y, Z)
                    best_conf = conf
                    best_cls = cls_name

                # markers at true XYZ
                p = Pose()
                p.position.x = float(X)
                p.position.y = float(Y)
                p.position.z = float(Z)
                p.orientation.w = 1.0

                m = Marker()
                m.header.stamp = self.get_clock().now().to_msg()
                m.header.frame_id = self.frame_id
                m.ns = "cubes"
                m.id = i
                m.type = Marker.SPHERE
                m.action = Marker.ADD
                m.pose = p
                m.scale.x = 0.06
                m.scale.y = 0.06
                m.scale.z = 0.06
                m.color.a = 1.0
                markers.markers.append(m)

                t = Marker()
                t.header = m.header
                t.ns = "cube_text"
                t.id = 1000 + i
                t.type = Marker.TEXT_VIEW_FACING
                t.action = Marker.ADD
                t.pose = p
                t.pose.position.z += 0.10
                t.scale.z = 0.08
                t.color.a = 1.0
                t.text = f"{cls_name} {conf:.2f}\nZ={Z:.2f}m"
                markers.markers.append(t)

        # Publish markers
        self.pub_markers.publish(markers)

        # Publish best target pose
        if best_xyz is not None:
            X, Y, Z = best_xyz
            ps = PoseStamped()
            ps.header.stamp = self.get_clock().now().to_msg()
            ps.header.frame_id = self.frame_id
            ps.pose.position.x = float(X)
            ps.pose.position.y = float(Y)
            ps.pose.position.z = float(Z)
            ps.pose.orientation.w = 1.0
            self.pub_target_pose.publish(ps)

        # Debug window
        if debug_frame is not None:
            cv2.imshow("Cube Detection", debug_frame)
            cv2.waitKey(1)

        # Optional JSON
        if self.publish_json:
            js = String()
            js.data = json.dumps(
                {
                    "stamp": time.time(),
                    "frame_id": self.frame_id,
                    "best": {"class": best_cls, "conf": round(best_conf, 4), "z": round(best_z, 4)}
                    if best_xyz is not None else None,
                    "detections": detections,
                },
                ensure_ascii=False,
            )
            self.pub_json.publish(js)


def main():
    rclpy.init()
    node = CubeDetectorStandardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()