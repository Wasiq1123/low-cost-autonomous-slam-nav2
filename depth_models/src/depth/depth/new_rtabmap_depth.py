# Runs an OpenVINO YOLO-based monocular depth estimation node that converts RGB camera frames into smoothed, scale-corrected 32-bit depth images and publishes them with matching camera information.

import time
import copy
import cv2
import numpy as np
import openvino as ov
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

MODEL_PATH = "/home/wasiq/Downloads/save_models/yolo_depth/lab_finetune_nano_v3_469_less_frozen_best.xml"


class DepthEstimationNode(Node):
    def __init__(self):
        super().__init__("depth_estimation_node")
        self.bridge = CvBridge()

        # Temporal median over recent frames kills flicker, not bias — see
        # scale_factor below for the actual measured correction.
        self.smoothing_window = 3
        self.depth_history = []

        # scale_factor: measured via 1m/2m target test, true/predicted ratio.
        # Leave at 1.0 only if it hasn't been measured yet.
        self.scale_factor = 1.0
        self.depth_clip_min = 0.2
        self.depth_clip_max = 4.0

        self.rgb_sub = self.create_subscription(Image, "/camera/color/image_raw", self.rgb_callback, 10)
        self.camera_info_sub = self.create_subscription(
            CameraInfo, "/camera/color/camera_info", self.camera_info_callback, 10
        )
        self.depth_pub = self.create_publisher(Image, "/camera/depth/image_raw", 10)
        self.depth_info_pub = self.create_publisher(CameraInfo, "/camera/depth/camera_info", 10)
        self.latest_camera_info = None

        self.core = ov.Core()
        self.get_logger().info("Loading OpenVINO depth model...")
        model = self.core.read_model(MODEL_PATH)

        self.input_port = model.inputs[0]
        self.input_name = self.input_port.get_any_name()
        input_shape = self.input_port.partial_shape

        if not input_shape.is_static:
            raise RuntimeError("Model input shape is dynamic — expected a static YOLO export.")

        b, c, h, w = input_shape.to_shape()
        if len([b, c, h, w]) != 4:
            raise RuntimeError(f"Expected 4D BCHW input, got {list(input_shape.to_shape())}")
        self.model_batch, self.model_channels, self.model_height, self.model_width = b, c, h, w
        self.get_logger().info(f"Input: batch={b} channels={c} height={h} width={w}")

        self.compiled_model = self.core.compile_model(model, "AUTO")
        self.get_logger().info(f"Running on: {self.compiled_model.get_property('EXECUTION_DEVICES')}")

        dummy = np.zeros((b, c, h, w), dtype=np.float32)
        self.compiled_model({self.input_name: dummy})
        self.get_logger().info("Warmup complete.")

    def camera_info_callback(self, msg):
        self.latest_camera_info = msg

    def preprocess(self, rgb):
        resized = cv2.resize(rgb, (self.model_width, self.model_height), interpolation=cv2.INTER_LINEAR)
        image = resized.astype(np.float32) / 255.0
        image = np.transpose(image, (2, 0, 1))
        image = np.expand_dims(image, axis=0)
        if self.model_batch > 1:
            image = np.repeat(image, self.model_batch, axis=0)
        return image

    def extract_depth(self, result):
        depth = np.asarray(result[self.compiled_model.outputs[0]])

        if depth.ndim == 4:
            depth = depth[0]
        elif depth.ndim == 3 and depth.shape[0] in (1, self.model_batch):
            depth = depth[0]

        if depth.ndim == 3:
            depth = depth[0] if depth.shape[0] == 1 else depth[:, :, 0]

        if depth.ndim != 2:
            raise RuntimeError(f"Could not reduce model output to 2D, got shape {depth.shape}")
        return depth

    def smooth_depth(self, depth):
        if self.depth_history and self.depth_history[-1].shape != depth.shape:
            self.get_logger().warning("Depth shape changed — resetting smoothing history.")
            self.depth_history = []

        self.depth_history.append(depth)
        self.depth_history = self.depth_history[-self.smoothing_window:]

        return np.median(np.stack(self.depth_history), axis=0).astype(np.float32)

    def rgb_callback(self, msg):
        try:
            rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
        except Exception as e:
            self.get_logger().error(f"CV Bridge error: {e}")
            return

        original_h, original_w = rgb.shape[:2]

        try:
            tensor = self.preprocess(rgb)
        except Exception as e:
            self.get_logger().error(f"Preprocessing error: {e}")
            return

        t0 = time.perf_counter()
        try:
            result = self.compiled_model({self.input_name: tensor})
        except Exception as e:
            self.get_logger().error(f"Inference error: {e}")
            return
        self.get_logger().info(f"Inference: {(time.perf_counter() - t0) * 1000:.1f} ms")

        try:
            depth = self.extract_depth(result)
        except Exception as e:
            self.get_logger().error(f"Depth extraction error: {e}")
            return

        depth = cv2.resize(depth, (original_w, original_h), interpolation=cv2.INTER_LINEAR).astype(np.float32)

        # Scale correction, then spatial median (kills single-frame spikes),
        # then mask out-of-range/garbage (never clip — that fabricates a
        # confident false reading at the boundary), then temporal median.
        depth *= self.scale_factor
        depth = cv2.medianBlur(depth, 5)
        invalid = (depth < self.depth_clip_min) | (depth > self.depth_clip_max) | ~np.isfinite(depth)
        depth[invalid] = 0.0
        depth = self.smooth_depth(depth)

        depth_msg = self.bridge.cv2_to_imgmsg(depth, encoding="32FC1")
        depth_msg.header = msg.header
        self.depth_pub.publish(depth_msg)

        if self.latest_camera_info is not None:
            info = copy.deepcopy(self.latest_camera_info)
            info.header = msg.header
            self.depth_info_pub.publish(info)


def main(args=None):
    rclpy.init(args=args)
    node = DepthEstimationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()