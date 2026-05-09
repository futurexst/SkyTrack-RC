#!/usr/bin/env python3

import csv
import os
import re
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float32, Int32, String


class MultiLoggerNode(Node):
    def __init__(self):
        super().__init__('multi_logger_node')

        log_dir = os.path.expanduser('~/lane_tracer_logs/csv')
        os.makedirs(log_dir, exist_ok=True)

        now_str = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.minimal_csv_path = os.path.join(log_dir, f'lane_minimal_{now_str}.csv')

        self.minimal_file = open(self.minimal_csv_path, 'w', newline='', encoding='utf-8')
        self.writer = csv.writer(self.minimal_file)

        # 최소 분석용 통합 로그
        self.writer.writerow([
            'pc_time',
            'ros_time_sec',

            # perception -> ESP32 topic data
            'target_error_topic',
            'search_cmd_topic',

            # perception/vision state, /perception/log_status가 있으면 채워짐
            'robot_center_x',
            'robot_center_y',
            'robot_heading_deg',
            'aruco_speed_px_s',
            'aruco_w_deg_s',
            'seed_x',
            'seed_y',
            'p1_x',
            'p1_y',
            'p2_x',
            'p2_y',
            'p3_x',
            'p3_y',
            'path_status',

            # ESP32 control debug
            'cam_err_esp',
            'steer',
            'left_pwm',
            'right_pwm',
            'valid',
            'esp_search',
            'lost',

            # IMU minimum
            'gyro_z',

            # raw strings for recovery/debug
            'raw_perception',
            'raw_esp_debug',
            'raw_imu',
        ])
        self.minimal_file.flush()

        # latest state cache
        self.target_error_topic = ''
        self.search_cmd_topic = ''

        self.perception = {
            'robot_center_x': '',
            'robot_center_y': '',
            'robot_heading_deg': '',
            'aruco_speed_px_s': '',
            'aruco_w_deg_s': '',
            'seed_x': '',
            'seed_y': '',
            'p1_x': '',
            'p1_y': '',
            'p2_x': '',
            'p2_y': '',
            'p3_x': '',
            'p3_y': '',
            'path_status': '',
            'raw': '',
        }

        self.esp = {
            'cam_err': '',
            'steer': '',
            'left_pwm': '',
            'right_pwm': '',
            'valid': '',
            'search': '',
            'lost': '',
            'raw': '',
        }

        self.imu = {
            'gz': '',
            'raw': '',
        }

        qos_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # ESP32에서 이미 나오는 로그
        self.debug_sub = self.create_subscription(
            String,
            '/esp32/debug_status',
            self.debug_callback,
            qos_best_effort
        )

        self.imu_sub = self.create_subscription(
            String,
            '/esp32/imu_debug',
            self.imu_callback,
            qos_best_effort
        )

        # perception -> ESP32로 실제 들어가는 topic 값도 별도로 기록
        self.target_error_sub = self.create_subscription(
            Float32,
            '/target_heading_error',
            self.target_error_callback,
            qos_best_effort
        )

        self.search_cmd_sub = self.create_subscription(
            Int32,
            '/perception/search_cmd',
            self.search_cmd_callback,
            qos_best_effort
        )

        # 선택 사항: perception_node에서 String으로 publish하면 aruco/waypoint 열이 채워짐
        # 예: RCx:123 | RCy:456 | Head:12.3 | Spd:33.2 | W:4.5 | Sx:... | Path:ACCEPT
        self.perception_log_sub = self.create_subscription(
            String,
            '/perception/log_status',
            self.perception_log_callback,
            qos_best_effort
        )

        # 20 Hz로 통합 CSV 한 줄씩 저장
        self.log_timer = self.create_timer(0.05, self.write_minimal_row)

        self.get_logger().info(f'Minimal CSV: {self.minimal_csv_path}')
        self.get_logger().info('Optional perception numeric topic: /perception/log_status')

    def now_times(self):
        pc_time = datetime.now().isoformat(timespec='milliseconds')
        ros_time = self.get_clock().now().nanoseconds / 1e9
        return pc_time, f'{ros_time:.6f}'

    def extract(self, pattern, text):
        match = re.search(pattern, text)
        return match.group(1) if match else ''

    def extract_any(self, patterns, text):
        for pattern in patterns:
            value = self.extract(pattern, text)
            if value != '':
                return value
        return ''

    def target_error_callback(self, msg: Float32):
        self.target_error_topic = f'{msg.data:.6f}'

    def search_cmd_callback(self, msg: Int32):
        self.search_cmd_topic = str(msg.data)

    def debug_callback(self, msg: String):
        raw = msg.data
        self.esp['cam_err'] = self.extract(r'CamErr:([-+]?\d*\.?\d+)', raw)
        self.esp['steer'] = self.extract(r'Steer:([-+]?\d*\.?\d+)', raw)
        self.esp['left_pwm'] = self.extract(r'L:([-+]?\d+)', raw)
        self.esp['right_pwm'] = self.extract(r'R:([-+]?\d+)', raw)
        self.esp['valid'] = self.extract(r'Valid:(\d+)', raw)
        self.esp['search'] = self.extract(r'Search:([-+]?\d+)', raw)
        self.esp['lost'] = self.extract(r'Lost:(\d+)', raw)
        self.esp['raw'] = raw

    def imu_callback(self, msg: String):
        raw = msg.data
        self.imu['gz'] = self.extract(r'Gz:([-+]?\d*\.?\d+)', raw)
        self.imu['raw'] = raw

    def perception_log_callback(self, msg: String):
        raw = msg.data
        self.perception['raw'] = raw

        # 여러 표기 허용: RCx/RCX/robot_center_x 등
        self.perception['robot_center_x'] = self.extract_any([
            r'RCx:([-+]?\d*\.?\d+)',
            r'robot_center_x:([-+]?\d*\.?\d+)',
            r'Cx:([-+]?\d*\.?\d+)',
        ], raw)
        self.perception['robot_center_y'] = self.extract_any([
            r'RCy:([-+]?\d*\.?\d+)',
            r'robot_center_y:([-+]?\d*\.?\d+)',
            r'Cy:([-+]?\d*\.?\d+)',
        ], raw)
        self.perception['robot_heading_deg'] = self.extract_any([
            r'Head:([-+]?\d*\.?\d+)',
            r'Heading:([-+]?\d*\.?\d+)',
            r'robot_heading_deg:([-+]?\d*\.?\d+)',
        ], raw)
        self.perception['aruco_speed_px_s'] = self.extract_any([
            r'Spd:([-+]?\d*\.?\d+)',
            r'Speed:([-+]?\d*\.?\d+)',
            r'aruco_speed_px_s:([-+]?\d*\.?\d+)',
        ], raw)
        self.perception['aruco_w_deg_s'] = self.extract_any([
            r'W:([-+]?\d*\.?\d+)',
            r'Omega:([-+]?\d*\.?\d+)',
            r'aruco_w_deg_s:([-+]?\d*\.?\d+)',
        ], raw)

        self.perception['seed_x'] = self.extract_any([r'Sx:([-+]?\d*\.?\d+)', r'seed_x:([-+]?\d*\.?\d+)'], raw)
        self.perception['seed_y'] = self.extract_any([r'Sy:([-+]?\d*\.?\d+)', r'seed_y:([-+]?\d*\.?\d+)'], raw)
        self.perception['p1_x'] = self.extract_any([r'P1x:([-+]?\d*\.?\d+)', r'p1_x:([-+]?\d*\.?\d+)'], raw)
        self.perception['p1_y'] = self.extract_any([r'P1y:([-+]?\d*\.?\d+)', r'p1_y:([-+]?\d*\.?\d+)'], raw)
        self.perception['p2_x'] = self.extract_any([r'P2x:([-+]?\d*\.?\d+)', r'p2_x:([-+]?\d*\.?\d+)'], raw)
        self.perception['p2_y'] = self.extract_any([r'P2y:([-+]?\d*\.?\d+)', r'p2_y:([-+]?\d*\.?\d+)'], raw)
        self.perception['p3_x'] = self.extract_any([r'P3x:([-+]?\d*\.?\d+)', r'p3_x:([-+]?\d*\.?\d+)'], raw)
        self.perception['p3_y'] = self.extract_any([r'P3y:([-+]?\d*\.?\d+)', r'p3_y:([-+]?\d*\.?\d+)'], raw)
        self.perception['path_status'] = self.extract_any([r'Path:([^|]+)', r'path_status:([^|]+)'], raw).strip()

    def write_minimal_row(self):
        pc_time, ros_time = self.now_times()

        row = [
            pc_time,
            ros_time,

            self.target_error_topic,
            self.search_cmd_topic,

            self.perception['robot_center_x'],
            self.perception['robot_center_y'],
            self.perception['robot_heading_deg'],
            self.perception['aruco_speed_px_s'],
            self.perception['aruco_w_deg_s'],
            self.perception['seed_x'],
            self.perception['seed_y'],
            self.perception['p1_x'],
            self.perception['p1_y'],
            self.perception['p2_x'],
            self.perception['p2_y'],
            self.perception['p3_x'],
            self.perception['p3_y'],
            self.perception['path_status'],

            self.esp['cam_err'],
            self.esp['steer'],
            self.esp['left_pwm'],
            self.esp['right_pwm'],
            self.esp['valid'],
            self.esp['search'],
            self.esp['lost'],

            self.imu['gz'],

            self.perception['raw'],
            self.esp['raw'],
            self.imu['raw'],
        ]

        self.writer.writerow(row)
        self.minimal_file.flush()

    def destroy_node(self):
        if hasattr(self, 'minimal_file') and not self.minimal_file.closed:
            self.minimal_file.flush()
            self.minimal_file.close()

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MultiLoggerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Logger stopped.')
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
