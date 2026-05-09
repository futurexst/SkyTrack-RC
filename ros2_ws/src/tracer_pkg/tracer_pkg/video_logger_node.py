#!/usr/bin/env python3

import os
from datetime import datetime

import cv2
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class VideoLoggerNode(Node):
    def __init__(self):
        super().__init__('video_logger_node')

        log_dir = os.path.expanduser('~/lane_tracer_logs/videos')
        os.makedirs(log_dir, exist_ok=True)

        now_str = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.video_path = os.path.join(log_dir, f'perception_debug_{now_str}.mp4')

        self.bridge = CvBridge()
        self.writer = None
        self.fps = 30.0

        # 여기 토픽을 바꾸면 원본/디버그 영상 선택 가능
        self.image_topic = '/perception/debug_image'
        # self.image_topic = '/image_raw'

        self.subscription = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10
        )

        self.get_logger().info(f'Recording {self.image_topic} to: {self.video_path}')

    def image_callback(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        h, w = frame.shape[:2]

        if self.writer is None:
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            self.writer = cv2.VideoWriter(
                self.video_path,
                fourcc,
                self.fps,
                (w, h)
            )

            if not self.writer.isOpened():
                self.get_logger().error('VideoWriter를 열 수 없습니다.')
                return

        # PC 시각 overlay
        pc_time = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
        cv2.putText(
            frame,
            pc_time,
            (10, h - 15),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            2
        )

        self.writer.write(frame)

    def destroy_node(self):
        if self.writer is not None:
            self.writer.release()

        self.get_logger().info(f'Video saved: {self.video_path}')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VideoLoggerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Video logger stopped.')
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
