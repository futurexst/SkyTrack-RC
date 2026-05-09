#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        self.publisher_ = self.create_publisher(Image, '/real/image_raw', 10)

        self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        if not self.cap.isOpened():
            raise RuntimeError("웹캠을 열 수 없습니다. WSL usbipd 연결 상태를 확인하세요.")

        self.bridge = CvBridge()
        self.capture_fail_count = 0

        self.timer = self.create_timer(0.033, self.timer_callback)
        self.get_logger().info("웹캠 노드가 정상적으로 시작되었습니다. (/real/image_raw 발행 중...)")

    def timer_callback(self):
        ret, frame = self.cap.read()

        if not ret or frame is None:
            self.capture_fail_count += 1

            if self.capture_fail_count == 1 or self.capture_fail_count % 30 == 0:
                self.get_logger().warning(
                    f"이미지를 캡처하지 못했습니다. 실패 횟수: {self.capture_fail_count}"
                )
            return

        if self.capture_fail_count > 0:
            self.get_logger().info("카메라 캡처가 다시 정상화되었습니다.")
            self.capture_fail_count = 0

        # 카메라가 요청 해상도를 정확히 안 줄 때만 resize
        if frame.shape[1] != 640 or frame.shape[0] != 480:
            frame = cv2.resize(frame, (640, 480), interpolation=cv2.INTER_NEAREST)

        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_link'

        self.publisher_.publish(msg)

    def cleanup(self):
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()
        cv2.destroyAllWindows()
        self.get_logger().info("카메라 자원을 안전하게 해제했습니다.")


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = CameraNode()
        rclpy.spin(node)

    except KeyboardInterrupt:
        if node is not None:
            node.get_logger().info("사용자에 의해 카메라 노드가 종료됩니다.")

    except Exception as e:
        print(f"[camera_node] 시작 실패: {e}")

    finally:
        if node is not None:
            node.cleanup()
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
