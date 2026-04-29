#!/usr/bin/env python3

import cv2
import numpy as np
import rclpy
import torch

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
    # Bounding box coordinates in the original image:
    x1: int
    y1: int
    x2: int
    y2: int


class YoloAnnotatorNode(Node):
    def __init__(self) -> None:
        super().__init__("yolo_annotator")

        # Declare and get ROS parameters
        self.model_name = (
            self.declare_parameter("model", "yolo11n.pt")
            .get_parameter_value()
            .string_value
        )
        self.conf_threshold = (
            self.declare_parameter("conf_threshold", 0.2)
            .get_parameter_value()
            .double_value
        )
        self.iou_threshold = (
            self.declare_parameter("iou_threshold", 0.7)
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
        if self.allowed_cls:
            self.get_logger().info(f"You've chosen to keep these class IDs: {self.allowed_cls}")
        else:
            self.get_logger().warn("No allowed classes matched the model's class list.")

        # Create publisher and subscribers
        self.bridge = CvBridge()
        self.sub = self.create_subscription(
            Image, "/zed/zed_node/rgb/image_rect_color", self.on_image, 10)
        self.pub = self.create_publisher(
            Image, "/yolo/annotated_image", 10)
        self.detected_object_pub = self.create_publisher(
            String, "/yolo/detected_object", 10)

        self.parking_meter_px_pub = self.create_publisher(
            ConeLocationPixel, "/relative_parking_meter_px", 10)
            
        self.person_px_pub = self.create_publisher(
            ConeLocationPixel, "/relative_person_px", 10)
        
        self.stop_sign_px_pub = self.create_publisher(
            ConeLocationPixel, "/relative_stop_sign_px", 10)
        
        self.traffic_light_px_pub = self.create_publisher(
            ConeLocationPixel, "/relative_traffic_light_px", 10)

        self.traffic_light_crop_pub = self.create_publisher(
            Image, "/traffic_light_crop", 10)

    def get_class_color_map(self) -> dict[str, tuple[int, int, int]]:
        """
        Return a dictionary mapping a list of COCO class names you want to keep
        to the detection BGR colors in the annotated image. COCO class names include
        "chair", "couch", "tv", "laptop", "dining table", and many more. The list
        of available classes can be found in `self.model.names`.
        """
        # TODO: Customize this dictionary for the lab. Choose a subset of
        #       COCO class names to detect and their corresponding colors
        #       in the annotated image.
        return {
            "person": (0, 255, 0),
            "parking meter": (255, 0, 0),
            "stop sign": (0, 0, 255),
            "traffic light": (255, 255, 0),
            }

    def on_image(self, msg: Image) -> None:
        # Convert ROS -> OpenCV (BGR)
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"cv_bridge conversion failed: {e}")
            return

        # Run YOLO inference
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

        if not results:
            return

        # Convert results to Detection List
        dets = self.results_to_detections(results[0])
        
        self.publish_traffic_light_crop(bgr, dets, msg.header)
        
        #publish bottom pixel
        self.publish_detected_object_px(dets)
        
        # Publish detected object name
        self.publish_detected_object(dets)

        # Draw detections on BGR image
        annotated = self.draw_detections(bgr, dets)

        # Publish annotated BGR image
        out_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
        out_msg.header = msg.header
        self.pub.publish(out_msg)
        
    def publish_detected_object_px(self, detections: List[Detection]) -> None:
        class_to_pub = {
            "parking meter": self.parking_meter_px_pub,
            "person": self.person_px_pub,
            "stop sign": self.stop_sign_px_pub,
        }

        for class_name, publisher in class_to_pub.items():
            matching_detections = [
                det for det in detections
                if det.class_name == class_name
            ]

            if len(matching_detections) == 0:
                continue

            best = max(matching_detections, key=lambda det: det.confidence)

            u = (best.x1 + best.x2) // 2
            v = best.y2

            msg = ConeLocationPixel()
            msg.u = float(u)
            msg.v = float(v)

            publisher.publish(msg)

            self.get_logger().info(
                f"{class_name} pixel: u={u}, v={v}, conf={best.confidence:.2f}"
            )
    def publish_traffic_light_crop(self, bgr_image: np.ndarray, detections: List[Detection], header) -> None:
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

        self.get_logger().info(
            f"Detected object: {best_detection.class_name}, "
            f"confidence={best_detection.confidence:.2f}"
        )

    def results_to_detections(self, result) -> List[Detection]:
        """
        Convert an Ultralytics result into a Detection list.

        YOLOv11 outputs:
          result.boxes.xyxy: (N, 4) tensor
          result.boxes.conf: (N,) tensor
          result.boxes.cls:  (N,) tensor
        """
        detections = []

        if result.boxes is None:
            return detections

        xyxy = result.boxes.xyxy
        conf = result.boxes.conf
        cls = result.boxes.cls

        # Convert Torch tensors -> CPU numpy
        xyxy_np = xyxy.detach().cpu().numpy() if hasattr(xyxy, "detach") else np.asarray(xyxy)
        conf_np = conf.detach().cpu().numpy() if hasattr(conf, "detach") else np.asarray(conf)
        cls_np = cls.detach().cpu().numpy() if hasattr(cls, "detach") else np.asarray(cls)

        # TODO: Store YOLO outputs as Detections. Iterate through xyxy_np, conf_np, and cls_np
        #       to append a Detection with all its instance variables filled in to the
        #       detections List.
        #
        # Hint: use Python's zip keyword to iterate through the three arrays in a single for loop.
        for box, conf, cls in zip(xyxy_np, conf_np, cls_np):
            x1, y1, x2, y2 = box
            class_name = self.model.names[int(cls)]
            detections.append(
                Detection(
                    class_id=int(cls),
                    class_name=class_name,
                    confidence=float(conf),
                    x1=int(x1),
                    y1=int(y1),
                    x2=int(x2),
                    y2=int(y2),

                )
            )
        return detections

    def draw_detections(
        self,
        bgr_image: np.ndarray,
        detections: List[Detection],
    ) -> np.ndarray:

        out_image = bgr_image.copy()

        for det in detections:
            # TODO: Get the bounding box for the detection

            # TODO: Draw the bounding box around the detection to the output image.
            #       Use the colors you specified per class in `get_class_color_map`
            #       by accessing the self.class_color_map dictionary.
            #
            # Hint: Use cv2's `rectangle` function to draw a rectangle on the annotated image.

            # TODO: Label the box with the class name and confidence.
            #
            # Hint: Use cv2's `putText` function to put text on the annotated image.

            # Get bounding box
            x1, y1, x2, y2 = det.x1, det.y1, det.x2, det.y2

            # Get color for this class, or use a default color if not found
            color = self.class_color_map.get(det.class_name, (255, 255, 255))

            # Draw bounding box
            cv2.rectangle(
                out_image,
                (x1, y1),
                (x2, y2),
                color,
                2
            )

            # Create label text
            label = f"{det.class_name} {det.confidence:.2f}"

            # Draw label text
            cv2.putText(
                out_image,
                label,
                (x1, max(y1 - 10, 0)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                color,
                2
            )
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

