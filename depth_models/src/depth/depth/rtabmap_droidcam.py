# Captures 640×480 video from the phone camera via V4L2, publishes RGB images and camera calibration information to ROS 2, and maintains synchronized timestamps and camera_link frame IDs.

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge


class DroidCamPublisher(Node):
    def __init__(self):
        super().__init__("droidcam_publisher")
        self.bridge = CvBridge()
        self.frame_id = "camera_link"

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.image_pub = self.create_publisher(Image, "/camera/color/image_raw", qos)
        self.info_pub = self.create_publisher(CameraInfo, "/camera/color/camera_info", qos)

        self.cap = cv2.VideoCapture("/dev/video0", cv2.CAP_V4L2)
        if not self.cap.isOpened():
            raise RuntimeError("Failed to open /dev/video0")

        # 640x480 matches the phone's 4:3 sensor ratio and is divisible by 32 for YOLO.
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)

        self.width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.fps = self.cap.get(cv2.CAP_PROP_FPS)
        self.get_logger().info(f"Camera opened ({self.width}x{self.height}) @ {self.fps:.1f} FPS")

        if (self.width, self.height) != (640, 480):
            self.get_logger().warning(
                f"Requested 640x480, got {self.width}x{self.height} — "
                f"intrinsics below are wrong until rescaled."
            )

        # Estimated from sensor-size assumption (Tecno Spark, not a real calibration).
        fx, fy = 401.0, 403.5
        cx, cy = self.width / 2.0, self.height / 2.0

        self.camera_info = CameraInfo()
        self.camera_info.header.frame_id = self.frame_id
        self.camera_info.width = self.width
        self.camera_info.height = self.height
        self.camera_info.distortion_model = "plumb_bob"
        self.camera_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.camera_info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        self.camera_info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        self.camera_info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]

        self.timer = self.create_timer(1.0 / 30.0, self.publish_frame)

    def publish_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warning("Failed to read frame.")
            return

        stamp = self.get_clock().now().to_msg()

        image_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        image_msg.header.stamp = stamp
        image_msg.header.frame_id = self.frame_id
        self.image_pub.publish(image_msg)

        self.camera_info.header.stamp = stamp
        self.info_pub.publish(self.camera_info)

    def destroy_node(self):
        if self.cap.isOpened():
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DroidCamPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()