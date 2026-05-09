#!/usr/bin/env python3
"""
sim_controller_node_matched.py

Simulation controller that mirrors the ESP32 main.c split-error control logic.

Input topics from sim_perception_node_matched.py:
  /sim/path/lateral_error       std_msgs/Float32   [px]
  /sim/path/heading_error       std_msgs/Float32   [deg]
  /sim/path/lookahead_error     std_msgs/Float32   [deg]
  /sim/perception/search_cmd    std_msgs/Int32     0, 10, 15

Output:
  /cmd_vel                      geometry_msgs/Twist
  /sim/esp32/debug_status       std_msgs/String

Important:
  The real ESP32 outputs left/right PWM. Gazebo usually consumes /cmd_vel.
  Therefore this node computes the same PWM command as main.c first, then converts
  the PWM pair to differential-drive velocity using pwm_to_mps and wheel_base.
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Int32, String


class SimControllerNode(Node):
    def __init__(self):
        super().__init__('sim_controller_node')

        # =====================
        # ROS I/O
        # =====================
        self.lateral_sub = self.create_subscription(
            Float32,
            '/sim/path/lateral_error',
            self.lateral_callback,
            10,
        )
        self.heading_sub = self.create_subscription(
            Float32,
            '/sim/path/heading_error',
            self.heading_callback,
            10,
        )
        self.lookahead_sub = self.create_subscription(
            Float32,
            '/sim/path/lookahead_error',
            self.lookahead_callback,
            10,
        )
        self.search_sub = self.create_subscription(
            Int32,
            '/sim/perception/search_cmd',
            self.search_callback,
            10,
        )

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.debug_pub = self.create_publisher(String, '/sim/esp32/debug_status', 10)

        # =====================
        # main.c control parameters
        # =====================
        self.STEER_SIGN = 1.0

        self.MIN_DRIVE_PWM = 60
        self.START_KICK_PWM = 180
        self.START_KICK_TIME = 0.100

        self.SEARCH_TURN_PWM = 160
        self.SEARCH_CMD_TIMEOUT = 0.500

        # Lost-recovery uses Kp exactly like main.c.
        self.Kp = 1.05
        self.Ki = 0.0
        self.Kd = 0.0

        # Split path-error gains.
        self.Ky = 0.035
        self.Ktheta = 0.85
        self.Klook = 0.55
        self.Ktheta_high = 2.20

        self.HEADING_PRIORITY_LOW_DEG = 12.0
        self.HEADING_PRIORITY_HIGH_DEG = 30.0
        self.STEER_SLEW_RATE_PER_SEC = 450.0
        self.STEER_FILTER_ALPHA = 0.28
        self.HEADING_SLOWDOWN_GAIN = 10.0

        self.base_speed = 95
        self.STEER_LIMIT = 85.0
        self.CURVE_SLOWDOWN_GAIN = 0.35
        self.TRACKING_BASE_MIN = self.MIN_DRIVE_PWM

        self.TARGET_TIMEOUT = 0.250
        self.LOST_RECOVERY_BASE_PWM = 140
        self.LOST_RECOVERY_TIME = 0.400

        # =====================
        # Simulation conversion parameters
        # =====================
        # base_speed=150 PWM roughly maps to 0.16 m/s by default.
        self.declare_parameter('pwm_to_mps', 0.16 / 150.0)
        self.declare_parameter('wheel_base_m', 0.20)
        self.declare_parameter('linear_scale', 1.0)
        self.declare_parameter('angular_scale', 1.0)
        self.declare_parameter('invert_linear', False)
        self.declare_parameter('invert_angular', False)
        self.declare_parameter('max_linear_mps', 0.35)
        self.declare_parameter('max_angular_radps', 3.0)

        self.pwm_to_mps = float(self.get_parameter('pwm_to_mps').value)
        self.wheel_base_m = float(self.get_parameter('wheel_base_m').value)
        self.linear_scale = float(self.get_parameter('linear_scale').value)
        self.angular_scale = float(self.get_parameter('angular_scale').value)
        self.invert_linear = bool(self.get_parameter('invert_linear').value)
        self.invert_angular = bool(self.get_parameter('invert_angular').value)
        self.max_linear_mps = float(self.get_parameter('max_linear_mps').value)
        self.max_angular_radps = float(self.get_parameter('max_angular_radps').value)

        # =====================
        # Control state, mirroring main.c
        # =====================
        self.path_lateral_error_px = 0.0
        self.path_heading_error_deg = 0.0
        self.path_lookahead_error_deg = 0.0

        self.lateral_error_valid = False
        self.heading_error_valid = False
        self.lookahead_error_valid = False
        self.target_error_valid = False

        now = self.now_sec()
        self.last_lateral_error_time = 0.0
        self.last_heading_error_time = 0.0
        self.last_lookahead_error_time = 0.0
        self.last_search_cmd_time = 0.0
        self.last_control_time = now

        self.search_cmd = 0
        self.camera_heading_error = 0.0  # debug compatibility: mirrors lookahead error
        self.last_valid_error = 0.0
        self.prev_err = 0.0
        self.integral_err = 0.0
        self.last_deriv_err = 0.0

        self.filtered_steer = 0.0
        self.last_heading_priority = 0.0
        self.last_active_base_speed = self.base_speed
        self.last_steer = 0.0
        self.last_left_pwm = 0
        self.last_right_pwm = 0

        self.was_stopped = True
        self.start_kick_active = False
        self.start_kick_start_time = 0.0

        self.lost_recovery_active = False
        self.lost_recovery_start_time = 0.0

        self.timer = self.create_timer(0.01, self.control_loop)  # close to ESP timer behavior

        self.get_logger().info(
            'sim_controller_node matched to ESP32 main.c. '
            '/sim/path/* + /sim/perception/search_cmd -> /cmd_vel'
        )

    # =====================
    # Helpers
    # =====================
    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    @staticmethod
    def clamp(x, lo, hi):
        return max(lo, min(hi, x))

    @staticmethod
    def smoothstep01(x):
        x = max(0.0, min(1.0, x))
        return x * x * (3.0 - 2.0 * x)

    @staticmethod
    def slew_limit(target, current, max_delta):
        delta = target - current
        delta = max(-max_delta, min(max_delta, delta))
        return current + delta

    def apply_min_drive_pwm(self, pwm: int) -> int:
        if pwm <= 0:
            return 0
        if pwm < self.MIN_DRIVE_PWM:
            return self.MIN_DRIVE_PWM
        return pwm

    def stop_robot(self):
        self.last_left_pwm = 0
        self.last_right_pwm = 0
        self.last_steer = 0.0
        self.filtered_steer = 0.0
        self.last_heading_priority = 0.0
        self.was_stopped = True
        self.start_kick_active = False
        self.start_kick_start_time = 0.0
        self.lost_recovery_active = False
        self.lost_recovery_start_time = 0.0
        self.publish_twist_from_pwm(0, 0)

    def publish_twist_from_pwm(self, left_pwm: int, right_pwm: int):
        left_v = float(left_pwm) * self.pwm_to_mps
        right_v = float(right_pwm) * self.pwm_to_mps

        linear = 0.5 * (left_v + right_v)
        angular = (right_v - left_v) / max(self.wheel_base_m, 1e-6)

        linear *= self.linear_scale
        angular *= self.angular_scale

        if self.invert_linear:
            linear = -linear
        if self.invert_angular:
            angular = -angular

        linear = self.clamp(linear, -self.max_linear_mps, self.max_linear_mps)
        angular = self.clamp(angular, -self.max_angular_radps, self.max_angular_radps)

        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.cmd_pub.publish(twist)

    # =====================
    # ROS callbacks
    # =====================
    def lateral_callback(self, msg: Float32):
        self.path_lateral_error_px = float(msg.data)
        self.last_lateral_error_time = self.now_sec()
        self.lateral_error_valid = True
        self.target_error_valid = (
            self.lateral_error_valid and self.heading_error_valid and self.lookahead_error_valid
        )

    def heading_callback(self, msg: Float32):
        self.path_heading_error_deg = float(msg.data)
        self.last_heading_error_time = self.now_sec()
        self.heading_error_valid = True
        self.target_error_valid = (
            self.lateral_error_valid and self.heading_error_valid and self.lookahead_error_valid
        )

    def lookahead_callback(self, msg: Float32):
        self.path_lookahead_error_deg = float(msg.data)
        self.camera_heading_error = self.path_lookahead_error_deg
        self.last_lookahead_error_time = self.now_sec()
        self.lookahead_error_valid = True
        self.target_error_valid = (
            self.lateral_error_valid and self.heading_error_valid and self.lookahead_error_valid
        )

    def search_callback(self, msg: Int32):
        self.search_cmd = int(msg.data)
        self.last_search_cmd_time = self.now_sec()

    # =====================
    # Main control loop
    # =====================
    def control_loop(self):
        now = self.now_sec()

        # --------------------------------------------------------
        # 1) Marker recovery mode has highest priority.
        # main.c: 10 -> L=+160, R=-160 / 15 -> L=-160, R=+160
        # --------------------------------------------------------
        search_cmd_recent = (now - self.last_search_cmd_time) <= self.SEARCH_CMD_TIMEOUT
        if not search_cmd_recent:
            self.search_cmd = 0

        if search_cmd_recent and self.search_cmd == 10:
            left_pwm = self.SEARCH_TURN_PWM
            right_pwm = -self.SEARCH_TURN_PWM
            self.last_steer = 0.0
            self.last_left_pwm = left_pwm
            self.last_right_pwm = right_pwm
            self.was_stopped = False
            self.start_kick_active = False
            self.filtered_steer = 0.0
            self.last_heading_priority = 0.0
            self.last_control_time = now
            self.publish_twist_from_pwm(left_pwm, right_pwm)
            self.publish_debug(valid=True, search=True, lost=False)
            return

        if search_cmd_recent and self.search_cmd == 15:
            left_pwm = -self.SEARCH_TURN_PWM
            right_pwm = self.SEARCH_TURN_PWM
            self.last_steer = 0.0
            self.last_left_pwm = left_pwm
            self.last_right_pwm = right_pwm
            self.was_stopped = False
            self.start_kick_active = False
            self.filtered_steer = 0.0
            self.last_heading_priority = 0.0
            self.last_control_time = now
            self.publish_twist_from_pwm(left_pwm, right_pwm)
            self.publish_debug(valid=True, search=True, lost=False)
            return

        # --------------------------------------------------------
        # 2) Before all 3 split errors are seen, do not recover; just stop.
        # --------------------------------------------------------
        if not self.target_error_valid:
            self.integral_err = 0.0
            self.prev_err = 0.0
            self.last_deriv_err = 0.0
            self.lost_recovery_active = False
            self.lost_recovery_start_time = 0.0
            self.last_control_time = now
            self.stop_robot()
            self.publish_debug(valid=False, search=False, lost=False)
            return

        # --------------------------------------------------------
        # 3) Lost recovery, same order as main.c.
        # --------------------------------------------------------
        if self.lost_recovery_active:
            lost_elapsed = now - self.lost_recovery_start_time

            if lost_elapsed <= self.LOST_RECOVERY_TIME:
                alpha = 1.0 - (lost_elapsed / self.LOST_RECOVERY_TIME)
                alpha = self.clamp(alpha, 0.0, 1.0)
                recovery_error = self.last_valid_error * alpha

                recovery_steer = self.STEER_SIGN * (self.Kp * recovery_error)
                recovery_steer = self.clamp(recovery_steer, -self.STEER_LIMIT, self.STEER_LIMIT)

                left_pwm = self.LOST_RECOVERY_BASE_PWM - int(recovery_steer)
                right_pwm = self.LOST_RECOVERY_BASE_PWM + int(recovery_steer)

                left_pwm = self.clamp(left_pwm, 1, 255)
                right_pwm = self.clamp(right_pwm, 1, 255)
                left_pwm = self.apply_min_drive_pwm(left_pwm)
                right_pwm = self.apply_min_drive_pwm(right_pwm)

                self.last_steer = recovery_steer
                self.last_left_pwm = int(left_pwm)
                self.last_right_pwm = int(right_pwm)
                self.last_control_time = now
                self.prev_err = recovery_error
                self.last_deriv_err = 0.0

                self.publish_twist_from_pwm(int(left_pwm), int(right_pwm))
                self.publish_debug(valid=False, search=False, lost=True)
                return

            self.lost_recovery_active = False
            self.lost_recovery_start_time = 0.0
            self.integral_err = 0.0
            self.prev_err = self.camera_heading_error
            self.last_deriv_err = 0.0

            split_recent_after_recovery = self.split_error_recent(now)
            if not split_recent_after_recovery:
                self.target_error_valid = False
                self.stop_robot()
                self.last_control_time = now
                self.publish_debug(valid=False, search=False, lost=True)
                return

        # --------------------------------------------------------
        # 4) If any split error is stale, enter lost recovery.
        # --------------------------------------------------------
        if not self.split_error_recent(now):
            self.lost_recovery_active = True
            self.lost_recovery_start_time = now
            self.integral_err = 0.0
            self.prev_err = self.last_valid_error
            self.last_deriv_err = 0.0
            self.last_control_time = now
            self.publish_debug(valid=False, search=False, lost=True)
            return

        # --------------------------------------------------------
        # 5) Normal split-error tracking.
        # --------------------------------------------------------
        dt = now - self.last_control_time
        self.last_control_time = now
        if dt <= 0.0 or dt > 0.1:
            dt = 0.01

        e_y = self.path_lateral_error_px
        e_theta = self.path_heading_error_deg
        e_look = self.path_lookahead_error_deg

        abs_heading = abs(e_theta)
        priority_raw = (
            (abs_heading - self.HEADING_PRIORITY_LOW_DEG)
            / (self.HEADING_PRIORITY_HIGH_DEG - self.HEADING_PRIORITY_LOW_DEG)
        )
        heading_priority = self.smoothstep01(priority_raw)

        normal_steer = (self.Ky * e_y) + (self.Ktheta * e_theta) + (self.Klook * e_look)
        heading_steer = self.Ktheta_high * e_theta
        steer_cmd = ((1.0 - heading_priority) * normal_steer) + (heading_priority * heading_steer)

        # For lost recovery/debug: keep an equivalent scalar error.
        error = steer_cmd
        self.integral_err = 0.0
        self.last_deriv_err = 0.0
        self.prev_err = error
        self.last_valid_error = error

        steer_cmd = self.STEER_SIGN * steer_cmd
        steer_cmd = self.clamp(steer_cmd, -self.STEER_LIMIT, self.STEER_LIMIT)

        filtered_target = (
            self.STEER_FILTER_ALPHA * steer_cmd
            + (1.0 - self.STEER_FILTER_ALPHA) * self.filtered_steer
        )
        max_delta = self.STEER_SLEW_RATE_PER_SEC * dt
        self.filtered_steer = self.slew_limit(filtered_target, self.filtered_steer, max_delta)

        steer = self.clamp(self.filtered_steer, -self.STEER_LIMIT, self.STEER_LIMIT)
        self.last_heading_priority = heading_priority

        active_base_speed = (
            self.base_speed
            - int(self.CURVE_SLOWDOWN_GAIN * abs(steer))
            - int(self.HEADING_SLOWDOWN_GAIN * heading_priority)
        )
        active_base_speed = int(self.clamp(active_base_speed, self.TRACKING_BASE_MIN, self.base_speed))
        self.last_active_base_speed = active_base_speed

        left_pwm = active_base_speed - int(steer)
        right_pwm = active_base_speed + int(steer)

        left_pwm = int(self.clamp(left_pwm, 1, 255))
        right_pwm = int(self.clamp(right_pwm, 1, 255))
        left_pwm = self.apply_min_drive_pwm(left_pwm)
        right_pwm = self.apply_min_drive_pwm(right_pwm)

        command_moving = left_pwm > 0 or right_pwm > 0
        if command_moving and self.was_stopped and not self.start_kick_active:
            self.start_kick_active = True
            self.start_kick_start_time = now

        if self.start_kick_active:
            kick_elapsed = now - self.start_kick_start_time
            if kick_elapsed <= self.START_KICK_TIME:
                if 0 < left_pwm < self.START_KICK_PWM:
                    left_pwm = self.START_KICK_PWM
                if 0 < right_pwm < self.START_KICK_PWM:
                    right_pwm = self.START_KICK_PWM
            else:
                self.start_kick_active = False
                self.was_stopped = False

        if not command_moving:
            self.was_stopped = True
            self.start_kick_active = False
            self.start_kick_start_time = 0.0
        elif not self.start_kick_active:
            self.was_stopped = False

        self.lost_recovery_active = False
        self.lost_recovery_start_time = 0.0

        self.last_steer = steer
        self.last_left_pwm = int(left_pwm)
        self.last_right_pwm = int(right_pwm)

        self.publish_twist_from_pwm(int(left_pwm), int(right_pwm))
        self.publish_debug(valid=True, search=False, lost=False)

    def split_error_recent(self, now: float) -> bool:
        return (
            self.lateral_error_valid
            and self.heading_error_valid
            and self.lookahead_error_valid
            and ((now - self.last_lateral_error_time) <= self.TARGET_TIMEOUT)
            and ((now - self.last_heading_error_time) <= self.TARGET_TIMEOUT)
            and ((now - self.last_lookahead_error_time) <= self.TARGET_TIMEOUT)
        )

    def publish_debug(self, valid=True, search=False, lost=False):
        msg = String()
        msg.data = (
            f"CamErr:{self.camera_heading_error:.2f} | "
            f"Lat:{self.path_lateral_error_px:.2f} | "
            f"Head:{self.path_heading_error_deg:.2f} | "
            f"Look:{self.path_lookahead_error_deg:.2f} | "
            f"LastErr:{self.last_valid_error:.2f} | "
            f"Steer:{self.last_steer:.2f} | "
            f"L:{self.last_left_pwm} | "
            f"R:{self.last_right_pwm} | "
            f"Valid:{1 if valid else 0} | "
            f"Ky:{self.Ky:.3f} | "
            f"Ktheta:{self.Ktheta:.2f} | "
            f"Klook:{self.Klook:.2f} | "
            f"HeadPri:{self.last_heading_priority:.2f} | "
            f"Base:{self.last_active_base_speed} | "
            f"Min:{self.MIN_DRIVE_PWM} | "
            f"Kick:{1 if self.start_kick_active else 0} | "
            f"Search:{self.search_cmd if search else 0} | "
            f"Lost:{1 if lost else 0}"
        )
        self.debug_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SimControllerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop = Twist()
        node.cmd_pub.publish(stop)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
