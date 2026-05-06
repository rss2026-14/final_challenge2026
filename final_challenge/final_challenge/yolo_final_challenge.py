#!/usr/bin/env python3

import cv2
import numpy as np
import rclpy
import torch
import time

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from dataclasses import dataclass
from rclpy.node import Node
from typing import List
from ultralytics import YOLO
from std_msgs.msg import String
from vs_msgs.msg import ConeLocationPixel


@dataclass(frozen=True)
class Detection:
    class_id: int
    class_name: str
    confidence: float
    x1: int
    y1: int
    x2: int
    y2: int


class YoloAnnotatorNode(Node):
    def __init__(self) -> None:
        super().__init__("yolo_annotator")

        self.model_name = (
            self.declare_parameter("model", "yolo11n.pt")
            .get_parameter_value()
            .string_value
        )
        self.conf_threshold = (
            self.declare_parameter("conf_threshold", 0.5)
            .get_parameter_value()
            .double_value
        )
        self.iou_threshold = (
            self.declare_parameter("iou_threshold", 0.1)
            .get_parameter_value()
            .double_value
        )
        self.image_queue_size = (
            self.declare_parameter("image_queue_size", 1)
            .get_parameter_value()
            .integer_value
        )
        self.inference_period_sec = (
            self.declare_parameter("inference_period_sec", 0.0)
            .get_parameter_value()
            .double_value
        )
        self.publish_annotated_image = (
            self.declare_parameter("publish_annotated_image", True)
            .get_parameter_value()
            .bool_value
        )
        self.publish_traffic_light_crop_enabled = (
            self.declare_parameter("publish_traffic_light_crop", True)
            .get_parameter_value()
            .bool_value
        )
        self.log_detections = (
            self.declare_parameter("log_detections", True)
            .get_parameter_value()
            .bool_value
        )
        self.stop_after_first_detection = (
            self.declare_parameter("stop_after_first_detection", False)
            .get_parameter_value()
            .bool_value
        )
        self.lock_target_class = (
            self.declare_parameter("lock_target_class", "")
            .get_parameter_value()
            .string_value
        ).strip()
        self.unload_model_after_lock = (
            self.declare_parameter("unload_model_after_lock", True)
            .get_parameter_value()
            .bool_value
        )

        # Multiplier used to estimate where the traffic light touches the ground.
        # This pushes the homography pixel below the YOLO traffic-light box.
        self.traffic_light_ground_offset_ratio = (
            self.declare_parameter("traffic_light_ground_offset_ratio", 13.0 / 16.5)
            .get_parameter_value()
            .double_value
        )

        self.device = "cuda:0" if torch.cuda.is_available() else "cpu"
        self.model = YOLO(self.model_name)
        self.model.to(self.device)

        self.class_color_map = self.get_class_color_map()
        self.allowed_cls = [
            i for i, name in self.model.names.items()
            if name in self.class_color_map
        ]

        self.get_logger().info(f"Running {self.model_name} on device {self.device}")
        self.get_logger().info(f"Confidence threshold: {self.conf_threshold}")
        self.get_logger().info(
            f"YOLO runtime options: queue={self.image_queue_size}, "
            f"period={self.inference_period_sec:.2f}s, "
            f"annotated={self.publish_annotated_image}, "
            f"stop_after_first_detection={self.stop_after_first_detection}, "
            f"lock_target_class='{self.lock_target_class}'"
        )
        self.get_logger().info(
            f"Traffic light ground offset ratio: {self.traffic_light_ground_offset_ratio:.3f}"
        )

        if self.allowed_cls:
            self.get_logger().info(
                f"You've chosen to keep these class IDs: {self.allowed_cls}"
            )
        else:
            self.get_logger().warn("No allowed classes matched the model's class list.")

        self.bridge = CvBridge()
        self.processing_image = False
        self.locked_detection = None
        self.last_inference_time = 0.0

        self.sub = self.create_subscription(
            Image,
            "/zed/zed_node/rgb/image_rect_color",
            self.on_image,
            self.image_queue_size,
        )

        self.pub = self.create_publisher(
            Image,
            "/yolo/annotated_image",
            10,
        )

        self.detected_object_pub = self.create_publisher(
            String,
            "/yolo/detected_object",
            10,
        )

        self.parking_meter_px_pub = self.create_publisher(
            ConeLocationPixel,
            "/relative_parking_meter_px",
            10,
        )

        self.traffic_light_px_pub = self.create_publisher(
            ConeLocationPixel,
            "/relative_traffic_light_px",
            10,
        )

        self.traffic_light_crop_pub = self.create_publisher(
            Image,
            "/traffic_light_crop",
            10,
        )

    def get_class_color_map(self) -> dict[str, tuple[int, int, int]]:
        return {
            "parking meter": (255, 0, 0),
            "traffic light": (255, 255, 0),
        }

    def on_image(self, msg: Image) -> None:
        if self.locked_detection is not None:
            return

        if self.processing_image:
            return

        now = time.monotonic()
        if now - self.last_inference_time < self.inference_period_sec:
            return

        self.processing_image = True

        try:
            try:
                bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            except Exception as e:
                self.get_logger().error(f"cv_bridge conversion failed: {e}")
                return

            try:
                results = self.model(
                    bgr,
                    classes=self.allowed_cls,
                    conf=self.conf_threshold,
                    iou=self.iou_threshold,
                    verbose=False,
                )
            except Exception as e:
                self.get_logger().error(f"YOLO inference failed: {e}")
                return

            self.last_inference_time = now

            if not results:
                return

            dets = self.results_to_detections(results[0])

            if self.log_detections:
                self.log_detection_summary(dets)

            if self.publish_traffic_light_crop_enabled:
                self.publish_traffic_light_crop(bgr, dets, msg.header)

            # Publish the ground-contact pixel estimate for homography.
            self.publish_detected_object_px(dets, bgr.shape)

            self.publish_detected_object(dets)

            if self.publish_annotated_image:
                annotated = self.draw_detections(bgr, dets)

                out_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
                out_msg.header = msg.header
                self.pub.publish(out_msg)

            self.maybe_lock_detection(dets)
        finally:
            self.processing_image = False

    def log_detection_summary(self, detections: List[Detection]) -> None:
        if len(detections) == 0:
            self.get_logger().info("YOLO detected nothing")
            return

        for det in detections:
            self.get_logger().info(
                f"YOLO detected {det.class_name}, "
                f"conf={det.confidence:.2f}, "
                f"box=({det.x1},{det.y1})-({det.x2},{det.y2})"
            )

    def maybe_lock_detection(self, detections: List[Detection]) -> None:
        if not self.stop_after_first_detection or len(detections) == 0:
            return

        candidates = detections
        if self.lock_target_class:
            candidates = [
                det for det in detections
                if det.class_name == self.lock_target_class
            ]

        if len(candidates) == 0:
            return

        self.locked_detection = max(candidates, key=lambda det: det.confidence)
        self.get_logger().warn(
            f"Locked YOLO on {self.locked_detection.class_name} "
            f"with confidence={self.locked_detection.confidence:.2f}; "
            "stopping image subscription."
        )

        self.destroy_subscription(self.sub)
        self.sub = None

        if self.unload_model_after_lock:
            self.model = None
            if torch.cuda.is_available():
                torch.cuda.empty_cache()

    def get_homography_pixel(self, det: Detection, image_shape) -> tuple[int, int]:
        """
        Returns the pixel that should be passed into homography.

        For parking meter:
            use bottom-center of YOLO box.

        For traffic light:
            YOLO usually detects only the light head, not the ground-contact point.
            So use center x, but push v downward by a fraction of the box height.
        """
        h, w = image_shape[:2]

        u = (det.x1 + det.x2) // 2

        if det.class_name == "traffic light":
            box_h = det.y2 - det.y1
            offset = int(self.traffic_light_ground_offset_ratio * box_h)
            v = det.y2 + offset
        else:
            v = det.y2

        u = max(0, min(w - 1, u))
        v = max(0, min(h - 1, v))

        return u, v

    def publish_detected_object_px(
        self,
        detections: List[Detection],
        image_shape,
    ) -> None:
        class_to_pub = {
            "parking meter": self.parking_meter_px_pub,
            "traffic light": self.traffic_light_px_pub,
        }

        for class_name, publisher in class_to_pub.items():
            matching_detections = [
                det for det in detections
                if det.class_name == class_name
            ]

            if len(matching_detections) == 0:
                continue

            best = max(matching_detections, key=lambda det: det.confidence)

            u, v = self.get_homography_pixel(best, image_shape)

            msg = ConeLocationPixel()
            msg.u = float(u)
            msg.v = float(v)

            publisher.publish(msg)

            if self.log_detections:
                self.get_logger().info(
                    f"{class_name} homography pixel: "
                    f"u={u}, v={v}, conf={best.confidence:.2f}, "
                    f"box=({best.x1},{best.y1})-({best.x2},{best.y2})"
                )

    def publish_traffic_light_crop(
        self,
        bgr_image: np.ndarray,
        detections: List[Detection],
        header,
    ) -> None:
        traffic_lights = [
            det for det in detections
            if det.class_name == "traffic light"
        ]

        if len(traffic_lights) == 0:
            return

        best = max(traffic_lights, key=lambda det: det.confidence)

        h, w = bgr_image.shape[:2]

        x1 = max(0, best.x1)
        y1 = max(0, best.y1)
        x2 = min(w - 1, best.x2)
        y2 = min(h - 1, best.y2)

        if x2 <= x1 or y2 <= y1:
            return

        crop = bgr_image[y1:y2, x1:x2]

        crop_msg = self.bridge.cv2_to_imgmsg(crop, encoding="bgr8")
        crop_msg.header = header
        self.traffic_light_crop_pub.publish(crop_msg)

        if self.log_detections:
            self.get_logger().info(
                f"Published traffic light crop, conf={best.confidence:.2f}"
            )

    def publish_detected_object(self, detections: List[Detection]) -> None:
        msg = String()

        if len(detections) == 0:
            msg.data = "none"
            self.detected_object_pub.publish(msg)
            return

        best_detection = max(detections, key=lambda det: det.confidence)

        msg.data = best_detection.class_name
        self.detected_object_pub.publish(msg)

        if self.log_detections:
            self.get_logger().info(
                f"Detected object: {best_detection.class_name}, "
                f"confidence={best_detection.confidence:.2f}"
            )

    def results_to_detections(self, result) -> List[Detection]:
        detections = []

        if result.boxes is None:
            return detections

        xyxy = result.boxes.xyxy
        conf = result.boxes.conf
        cls = result.boxes.cls

        xyxy_np = (
            xyxy.detach().cpu().numpy()
            if hasattr(xyxy, "detach")
            else np.asarray(xyxy)
        )
        conf_np = (
            conf.detach().cpu().numpy()
            if hasattr(conf, "detach")
            else np.asarray(conf)
        )
        cls_np = (
            cls.detach().cpu().numpy()
            if hasattr(cls, "detach")
            else np.asarray(cls)
        )

        for box, confidence, class_id in zip(xyxy_np, conf_np, cls_np):
            x1, y1, x2, y2 = box
            class_name = self.model.names[int(class_id)]

            detections.append(
                Detection(
                    class_id=int(class_id),
                    class_name=class_name,
                    confidence=float(confidence),
                    x1=int(x1),
                    y1=int(y1),
                    x2=int(x2),
                    y2=int(y2),
                )
            )

        return detections

    def draw_homography_point(
        self,
        image: np.ndarray,
        det: Detection,
    ) -> None:
        u, v = self.get_homography_pixel(det, image.shape)

        cv2.circle(image, (u, v), 7, (0, 0, 255), -1)

        cv2.line(
            image,
            (det.x1, v),
            (det.x2, v),
            (0, 0, 255),
            2,
        )

        cv2.line(
            image,
            (u, det.y1),
            (u, v),
            (0, 0, 255),
            2,
        )

        cv2.putText(
            image,
            f"H px=({u},{v})",
            (u + 10, max(v - 10, 20)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 0, 255),
            2,
        )

    def draw_detections(
        self,
        bgr_image: np.ndarray,
        detections: List[Detection],
    ) -> np.ndarray:
        out_image = bgr_image.copy()

        for det in detections:
            x1, y1, x2, y2 = det.x1, det.y1, det.x2, det.y2

            color = self.class_color_map.get(
                det.class_name,
                (255, 255, 255),
            )

            # Draw YOLO bounding box.
            # This box is intentionally NOT adjusted.
            # Only the red homography point is adjusted.
            cv2.rectangle(
                out_image,
                (x1, y1),
                (x2, y2),
                color,
                2,
            )

            label = f"{det.class_name} {det.confidence:.2f}"

            cv2.putText(
                out_image,
                label,
                (x1, max(y1 - 10, 0)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                color,
                2,
            )

            # Draw adjusted homography point.
            self.draw_homography_point(out_image, det)

        return out_image


def main() -> None:
    rclpy.init()
    node = YoloAnnotatorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
