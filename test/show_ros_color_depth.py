#!/usr/bin/env python3
import argparse
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ImageViewer(Node):
    def __init__(self, color_topic, depth_topic, depth_max_m=4.0):
        super().__init__("image_viewer")

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.bridge = CvBridge()
        self.color_topic = color_topic or ""
        self.depth_topic = depth_topic or ""
        self.depth_max_m = float(depth_max_m)

        if self.color_topic:
            self.create_subscription(Image, self.color_topic, self.on_color, qos)
        if self.depth_topic:
            self.create_subscription(Image, self.depth_topic, self.on_depth, qos)

        self.get_logger().info(f"Subscribed: color={self.color_topic or 'None'}, depth={self.depth_topic or 'None'}")

    def on_color(self, msg: Image):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            cv2.imshow(f"COLOR ({self.color_topic})", img)
            key = cv2.waitKey(1) & 0xFF
            if key == 27:  # ESC
                rclpy.shutdown()
            elif key == ord('s'):
                cv2.imwrite("color_frame.png", img)
                self.get_logger().info("Saved color_frame.png")
        except Exception as e:
            self.get_logger().error(f"color cv_bridge error: {e}")

    def on_depth(self, msg: Image):
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            if depth.dtype == np.uint16:
                depth_m = np.clip(depth.astype(np.float32) / 1000.0, 0.0, self.depth_max_m)
                depth_vis = (depth_m / self.depth_max_m * 255.0).astype(np.uint8)
            elif depth.dtype == np.float32:
                depth_m = np.clip(depth, 0.0, self.depth_max_m)
                depth_vis = (depth_m / self.depth_max_m * 255.0).astype(np.uint8)
            else:
                dmin, dmax = np.nanmin(depth), np.nanmax(depth)
                if np.isfinite(dmin) and np.isfinite(dmax) and dmax > dmin:
                    depth_vis = ((depth - dmin) / (dmax - dmin) * 255.0).astype(np.uint8)
                else:
                    depth_vis = np.zeros_like(depth, dtype=np.uint8)

            depth_color = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)
            cv2.imshow(f"DEPTH ({self.depth_topic})", depth_color)
            key = cv2.waitKey(1) & 0xFF
            if key == 27:
                rclpy.shutdown()
            elif key == ord('s'):
                cv2.imwrite("depth_frame.png", depth_color)
                self.get_logger().info("Saved depth_frame.png")
        except Exception as e:
            self.get_logger().error(f"depth cv_bridge error: {e}")

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--color", default="/camera/color/image_raw")
    parser.add_argument("--depth", default="/camera/depth/image_raw")
    parser.add_argument("--depth-max", type=float, default=4.0)
    args = parser.parse_args()

    rclpy.init()
    node = ImageViewer(args.color, args.depth, args.depth_max)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
