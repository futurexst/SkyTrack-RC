#!/usr/bin/env python3

import math

import cv2
import cv2.aruco as aruco
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Int32, String


class PerceptionNode(Node):
    def __init__(self):
        super().__init__('sim_perception_node')

        # =====================
        # ROS I/O
        # =====================
        self.subscription = self.create_subscription(
            Image,
            '/sim/image_raw',
            self.image_callback,
            10
        )

        # Split path-control errors are the only ESP32 control topics.

        # Split path-control errors for smoother and more interpretable control.
        self.lateral_error_publisher = self.create_publisher(
            Float32,
            '/sim/path/lateral_error',
            10
        )
        self.heading_error_publisher = self.create_publisher(
            Float32,
            '/sim/path/heading_error',
            10
        )
        self.lookahead_error_publisher = self.create_publisher(
            Float32,
            '/sim/path/lookahead_error',
            10
        )

        self.debug_image_publisher = self.create_publisher(
            Image,
            '/sim/perception/debug_image',
            10
        )

        # 이름은 유지. 실제로는 skeleton + waypoint debug image
        self.skeleton_debug_publisher = self.create_publisher(
            Image,
            '/sim/perception/roi_binary_image',
            10
        )

        self.search_cmd_publisher = self.create_publisher(
            Int32,
            '/sim/perception/search_cmd',
            10
        )

        # analysis/logging용: ArUco pose, speed, waypoint, path 상태를 문자열로 발행
        self.log_status_publisher = self.create_publisher(
            String,
            '/sim/perception/log_status',
            10
        )

        self.bridge = CvBridge()

        # =====================
        # ArUco marker ID
        # =====================
        self.left_marker_id = 10
        self.right_marker_id = 15

        # =====================
        # Search mode debounce
        # =====================
        self.search_debounce_frames = 4
        self.prev_raw_search_cmd = 0
        self.raw_search_count = 0
        self.debounced_search_cmd = 0

        # =====================
        # Binary threshold params
        # =====================
        self.adaptive_block_size = 71
        self.adaptive_c = 18
        self.open_kernel_size = 3
        self.close_kernel_size = 7

        # =====================
        # Robot removal mask: 사각형 2개
        # =====================
        self.front_mask_length_px = 55
        self.front_mask_width_px = 165
        self.front_mask_center_offset_px = 0.0

        self.body_mask_length_px = 62
        self.body_mask_width_px = 110
        self.body_mask_center_offset_px = -58

        # =====================
        # Waypoint traversal params
        # seed -> P1 -> P2 -> P3
        # =====================
        self.seed_waypoint_dist_px = 60.0
        self.seed_band_width_px = 30.0
        self.max_lateral_px = 105.0

        self.waypoint_step_px =60.0
        self.ring_band_px = 20.0

        self.total_chain_points = 4
        self.fit_waypoint_count = 3
        self.min_skeleton_pixels = 20

        # =====================
        # Pure-pursuit + lateral error control
        # =====================
        self.prev_chain_local = None
        self.prev_error_deg = 0.0

        # Component smoothing: keep perception output smooth before ESP control.
        # Larger alpha = more responsive, smaller alpha = smoother.
        self.error_smoothing_alpha = 0.28
        self.component_smoothing_alpha = 0.32
        self.prev_lookahead_error_deg = 0.0
        self.prev_lateral_error_px = 0.0
        self.prev_path_heading_error_deg = 0.0
        self.have_prev_components = False

        # 기존 P2 직접 추종 대신, filtered path 위의 lookahead point를 추종한다.
        # sim_controller_node는 /sim/path/* split error를 사용한다.
        self.lookahead_distance_px = 80.0
        self.lookahead_min_px = 60.0
        self.lookahead_max_px = 100.0

        # Log/debug용 scalar error. ROS topic으로는 보내지 않는다.
        # Split control gain은 ESP32 쪽에서 적용한다.
        self.k_lookahead = 1.00
        self.k_lateral = 0.00
        self.k_heading = 0.00
        self.control_error_limit_deg = 50.0

        # analysis/log/debug용으로 마지막 pursuit 계산값 저장
        self.last_lookahead_error_deg = None
        self.last_lateral_error_px = None
        self.last_path_heading_error_deg = None
        self.last_lookahead_point = None
        self.last_nearest_point = None

        # =====================
        # Seed constraints
        # =====================
        self.seed_min_dist_from_robot_px = 38.0
        self.seed_max_dist_from_robot_px = 105.0
        self.seed_max_lateral_px = 105.0
        self.seed_max_jump_px = 100.0
        self.seed_min_heading_dot = 0.00

        # Seed continuity lock.
        # accepted seed의 이미지 좌표 기준으로 다음 seed를 제한한다.
        # ACCEPT / fallback / SHIFT / RESET 이후 재획득에서도 이 lock을 유지한다.
        self.enable_seed_img_continuity_lock = True
        self.prev_seed_img_for_lock = None
        self.seed_img_max_jump_px = 75.0
        self.seed_img_score_weight = 1.00

        # 지나간 seed line을 mask로 지워서 다시 못 잡게 하는 기능.
        # 같은 트랙을 반복 주행해야 하므로 기본 OFF.
        self.enable_seed_history_blocking = False

        # =====================
        # Path lock / anti-research
        # =====================
        self.max_chain_jump_px = 75.0
        self.max_path_heading_jump_deg = 50.0

        self.path_hold_max_frames = 12
        self.path_hold_count = 0

        self.filtered_chain_img = None
        self.path_filter_alpha = 0.25

        self.path_locked = False
        self.last_path_status = "NO_PATH"

        # =====================
        # Perception log state
        # =====================
        self.prev_robot_center_for_log = None
        self.prev_robot_heading_for_log = None
        self.prev_log_time = None

        # =====================
        # Seed visited-line blocking
        # =====================
        # accepted seed들이 지나간 선을 기록하는 mask.
        # 이 mask 위에 있는 skeleton point는 seed 후보에서 제외.
        self.seed_history_mask = None
        self.prev_accepted_seed_img = None

        # 지나간 seed line 두께. 너무 얇으면 효과가 약하고,
        # 너무 두꺼우면 다음 seed를 못 잡을 수 있음.
        self.seed_history_thickness_px = 7

        # 이전 seed 점 근처도 막기 위한 반경
        self.seed_history_radius_px = 10

        # 현재 seed 바로 근처까지 막으면 다음 프레임 seed가 막힐 수 있어서,
        # 선을 현재 seed 직전까지만 그림.
        self.seed_history_release_px = 15

        # 너무 가까운 seed 이동은 history에 누적하지 않음.
        self.seed_history_min_step_px = 8

        # =====================
        # ArUco detector
        # =====================
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)

        if hasattr(aruco, "DetectorParameters_create"):
            self.aruco_params = aruco.DetectorParameters_create()
        else:
            self.aruco_params = aruco.DetectorParameters()

        self.aruco_params.adaptiveThreshConstant = 7
        self.aruco_params.minMarkerPerimeterRate = 0.005
        self.aruco_params.maxMarkerPerimeterRate = 4.0
        self.aruco_params.polygonalApproxAccuracyRate = 0.10
        self.aruco_params.minCornerDistanceRate = 0.005
        self.aruco_params.minDistanceToBorder = 1
        self.aruco_params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX

        self.get_logger().info(
            f'sim_perception_node pure-pursuit + lateral-error version started. OpenCV={cv2.__version__}'
        )

    # ============================================================
    # Utility
    # ============================================================
    def normalize_angle_deg(self, angle_deg: float) -> float:
        while angle_deg > 180.0:
            angle_deg -= 360.0
        while angle_deg < -180.0:
            angle_deg += 360.0
        return angle_deg

    def skeletonize(self, binary):
        """
        binary: 0 or 255 image
        return: skeleton image, 0 or 255
        """
        if hasattr(cv2, "ximgproc") and hasattr(cv2.ximgproc, "thinning"):
            return cv2.ximgproc.thinning(binary)

        img = binary.copy()
        img[img > 0] = 255

        skeleton = np.zeros_like(img)
        element = cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3))

        while True:
            eroded = cv2.erode(img, element)
            opened = cv2.morphologyEx(eroded, cv2.MORPH_OPEN, element)
            temp = cv2.subtract(eroded, opened)
            skeleton = cv2.bitwise_or(skeleton, temp)
            img = eroded.copy()

            if cv2.countNonZero(img) == 0:
                break

        return skeleton

    def get_basis_vectors(self, robot_heading_deg):
        theta = math.radians(robot_heading_deg)
        forward = np.array([math.cos(theta), math.sin(theta)], dtype=np.float32)
        left = np.array([-forward[1], forward[0]], dtype=np.float32)
        return forward, left

    # ============================================================
    # Robot pose from ArUco
    # ============================================================
    def detect_robot_pose(self, gray, debug_image):
        corners, ids, _ = aruco.detectMarkers(
            gray,
            self.aruco_dict,
            parameters=self.aruco_params
        )

        detected_ids_text = "IDs: None"
        if ids is not None:
            detected_ids_text = "IDs: " + ",".join(str(int(x[0])) for x in ids)

        cv2.putText(
            debug_image,
            detected_ids_text,
            (20, 230),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 0, 255),
            2
        )

        left_marker = None
        right_marker = None
        marker10_seen = False
        marker15_seen = False

        robot_center = None
        robot_heading_deg = None
        heading_end = None

        if ids is not None and len(ids) >= 1:
            aruco.drawDetectedMarkers(debug_image, corners, ids)

            for i in range(len(ids)):
                marker_id = int(ids[i][0])
                marker_center = np.mean(corners[i][0], axis=0).astype(int)

                if marker_id == self.left_marker_id:
                    left_marker = marker_center
                    marker10_seen = True
                    cv2.circle(debug_image, tuple(marker_center), 6, (0, 255, 255), -1)

                elif marker_id == self.right_marker_id:
                    right_marker = marker_center
                    marker15_seen = True
                    cv2.circle(debug_image, tuple(marker_center), 6, (255, 255, 0), -1)

            if left_marker is not None and right_marker is not None:
                robot_center = (
                    int((left_marker[0] + right_marker[0]) / 2),
                    int((left_marker[1] + right_marker[1]) / 2)
                )

                cv2.line(debug_image, tuple(left_marker), tuple(right_marker), (255, 0, 0), 2)
                cv2.circle(debug_image, robot_center, 6, (0, 0, 255), -1)

                marker_dx = right_marker[0] - left_marker[0]
                marker_dy = right_marker[1] - left_marker[1]

                # heading 화살표가 뒤를 보면 아래 두 줄 부호 반대로 바꾸면 됨
                heading_x = marker_dy
                heading_y = -marker_dx

                robot_heading_deg = math.degrees(math.atan2(heading_y, heading_x))
                robot_heading_deg = self.normalize_angle_deg(robot_heading_deg)

                arrow_len = 45
                hx = int(robot_center[0] + arrow_len * math.cos(math.radians(robot_heading_deg)))
                hy = int(robot_center[1] + arrow_len * math.sin(math.radians(robot_heading_deg)))
                heading_end = (hx, hy)

                cv2.arrowedLine(debug_image, robot_center, heading_end, (255, 255, 0), 2)

                cv2.putText(
                    debug_image,
                    f"Heading: {robot_heading_deg:.1f} deg",
                    (20, 35),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (255, 0, 0),
                    2
                )

        return {
            "robot_center": robot_center,
            "robot_heading_deg": robot_heading_deg,
            "left_marker": left_marker,
            "right_marker": right_marker,
            "heading_end": heading_end,
            "marker10_seen": marker10_seen,
            "marker15_seen": marker15_seen,
        }

    # ============================================================
    # Search command
    # ============================================================
    def publish_search_cmd(self, marker10_seen, marker15_seen):
        if marker10_seen and marker15_seen:
            raw_search_cmd = 0
        elif marker10_seen and not marker15_seen:
            raw_search_cmd = 10
        elif marker15_seen and not marker10_seen:
            raw_search_cmd = 15
        else:
            raw_search_cmd = -1

        if raw_search_cmd == self.prev_raw_search_cmd:
            self.raw_search_count += 1
        else:
            self.prev_raw_search_cmd = raw_search_cmd
            self.raw_search_count = 1

        self.debounced_search_cmd = 0

        if raw_search_cmd == 0:
            self.debounced_search_cmd = 0

        elif raw_search_cmd in (10, 15):
            if self.raw_search_count >= self.search_debounce_frames:
                self.debounced_search_cmd = raw_search_cmd
            else:
                self.debounced_search_cmd = 0

        else:
            self.debounced_search_cmd = 0

        search_cmd = Int32()
        search_cmd.data = self.debounced_search_cmd
        self.search_cmd_publisher.publish(search_cmd)

        return search_cmd.data

    # ============================================================
    # Binary extraction
    # ============================================================
    def extract_line_binary(self, gray):
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)

        binary = cv2.adaptiveThreshold(
            blurred,
            255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY_INV,
            self.adaptive_block_size,
            self.adaptive_c
        )

        kernel_open = np.ones((self.open_kernel_size, self.open_kernel_size), np.uint8)
        binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel_open)

        kernel_close = np.ones((self.close_kernel_size, self.close_kernel_size), np.uint8)
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel_close)

        return binary

    # ============================================================
    # Robot mask
    # ============================================================
    def make_oriented_rect(self, center_xy, heading_deg, length_px, width_px):
        forward, left = self.get_basis_vectors(heading_deg)

        center = np.array(center_xy, dtype=np.float32)
        half_l = length_px / 2.0
        half_w = width_px / 2.0

        corners_local = np.array([
            [ half_l,  half_w],
            [ half_l, -half_w],
            [-half_l, -half_w],
            [-half_l,  half_w],
        ], dtype=np.float32)

        corners_img = []
        for x_local, y_local in corners_local:
            p = center + x_local * forward + y_local * left
            corners_img.append([int(p[0]), int(p[1])])

        return np.array(corners_img, dtype=np.int32)

    def get_robot_mask_polygons(self, robot_center, robot_heading_deg):
        if robot_center is None or robot_heading_deg is None:
            return None, None

        forward, _ = self.get_basis_vectors(robot_heading_deg)
        rc_center = np.array(robot_center, dtype=np.float32)

        front_center = rc_center + self.front_mask_center_offset_px * forward
        front_poly = self.make_oriented_rect(
            front_center,
            robot_heading_deg,
            self.front_mask_length_px,
            self.front_mask_width_px
        )

        body_center = rc_center + self.body_mask_center_offset_px * forward
        body_poly = self.make_oriented_rect(
            body_center,
            robot_heading_deg,
            self.body_mask_length_px,
            self.body_mask_width_px
        )

        return front_poly, body_poly

    def remove_robot_from_binary(self, binary, robot_center, robot_heading_deg):
        clean = binary.copy()

        if robot_center is None or robot_heading_deg is None:
            return clean

        front_poly, body_poly = self.get_robot_mask_polygons(robot_center, robot_heading_deg)

        if front_poly is not None:
            cv2.fillPoly(clean, [front_poly], 0)

        if body_poly is not None:
            cv2.fillPoly(clean, [body_poly], 0)

        return clean

    # ============================================================
    # Seed history mask
    # ============================================================
    def ensure_seed_history_mask(self, image_shape):
        h, w = image_shape[:2]

        if self.seed_history_mask is None:
            self.seed_history_mask = np.zeros((h, w), dtype=np.uint8)
            return

        if self.seed_history_mask.shape[:2] != (h, w):
            self.seed_history_mask = np.zeros((h, w), dtype=np.uint8)
            self.prev_accepted_seed_img = None

    def get_seed_not_visited_mask(self, image_shape, xs, ys):
        """
        skeleton point들 중 seed history line 위에 있지 않은 점만 True.
        enable_seed_history_blocking=False이면 지나간 트랙을 지우지 않고 전체 후보를 허용한다.
        """
        if not self.enable_seed_history_blocking:
            return np.ones_like(xs, dtype=bool)

        self.ensure_seed_history_mask(image_shape)

        if self.seed_history_mask is None:
            return np.ones_like(xs, dtype=bool)

        blocked = self.seed_history_mask[ys, xs] > 0
        return ~blocked

    def update_seed_history_from_chain(self, chain_img, image_shape):
        """
        accepted chain의 seed를 seed history에 누적.
        enable_seed_history_blocking=False이면 아무 것도 지우지 않는다.
        """
        if chain_img is None or len(chain_img) < 1:
            return

        if not self.enable_seed_history_blocking:
            return

        self.ensure_seed_history_mask(image_shape)

        current_seed = np.array(chain_img[0], dtype=np.float32)

        if self.prev_accepted_seed_img is None:
            self.prev_accepted_seed_img = current_seed.copy()
            return

        prev_seed = self.prev_accepted_seed_img.copy()
        segment = current_seed - prev_seed
        dist = np.linalg.norm(segment)

        if dist < self.seed_history_min_step_px:
            return

        direction = segment / max(dist, 1e-6)

        # 현재 seed 근처는 남겨둬야 다음 프레임 seed가 바로 막히지 않음
        draw_end = current_seed - self.seed_history_release_px * direction

        if np.linalg.norm(draw_end - prev_seed) > 1.0:
            p0 = (int(prev_seed[0]), int(prev_seed[1]))
            p1 = (int(draw_end[0]), int(draw_end[1]))

            cv2.line(
                self.seed_history_mask,
                p0,
                p1,
                255,
                self.seed_history_thickness_px
            )

            cv2.circle(
                self.seed_history_mask,
                p0,
                self.seed_history_radius_px,
                255,
                -1
            )

        self.prev_accepted_seed_img = current_seed.copy()

    # ============================================================
    # Waypoint chain
    # seed -> P1 -> P2 -> P3
    # ============================================================
    def find_waypoint_chain_from_skeleton(self, skeleton, robot_center, robot_heading_deg):
        ys, xs = np.where(skeleton > 0)

        if len(xs) < self.min_skeleton_pixels:
            return None, None, None, None

        # seed history 위에 있는 skeleton point는 seed 후보에서 제외
        seed_not_visited_mask = self.get_seed_not_visited_mask(skeleton.shape, xs, ys)

        cx, cy = robot_center
        robot_np = np.array([cx, cy], dtype=np.float32)

        forward, left = self.get_basis_vectors(robot_heading_deg)

        pts = np.stack([xs, ys], axis=1).astype(np.float32)
        rel = pts - robot_np

        x_local_all = rel @ forward
        y_local_all = rel @ left

        chain_img = []
        chain_local = []

        # --------------------------------------------------------
        # 1) seed 찾기: robot_center와의 거리 제한 + visited line 제외
        # --------------------------------------------------------
        dist_from_robot = np.linalg.norm(rel, axis=1)
        dir_from_robot = rel / np.maximum(dist_from_robot[:, None], 1e-6)
        heading_dot = dir_from_robot @ forward

        # 이미지 좌표 기준 seed continuity lock.
        # 로봇 local 좌표는 로봇 heading이 바뀌면 같은 실제 점도 크게 달라질 수 있으므로,
        # accepted seed의 이미지 좌표 기준 거리 제한을 fallback에서도 절대 풀지 않는다.
        if (
            self.enable_seed_img_continuity_lock
            and self.prev_seed_img_for_lock is not None
        ):
            prev_seed_img = np.array(self.prev_seed_img_for_lock, dtype=np.float32)
            seed_img_jump_all = np.linalg.norm(pts - prev_seed_img, axis=1)
            seed_continuity_mask = seed_img_jump_all < self.seed_img_max_jump_px
        else:
            seed_img_jump_all = np.zeros_like(x_local_all, dtype=np.float32)
            seed_continuity_mask = np.ones_like(x_local_all, dtype=bool)

        seed_mask = (
            (np.abs(x_local_all - self.seed_waypoint_dist_px) < self.seed_band_width_px) &
            (np.abs(y_local_all) < self.seed_max_lateral_px) &
            (dist_from_robot > self.seed_min_dist_from_robot_px) &
            (dist_from_robot < self.seed_max_dist_from_robot_px) &
            (heading_dot > self.seed_min_heading_dot) &
            seed_not_visited_mask &
            seed_continuity_mask
        )

        # 이전 seed가 있으면 갑자기 멀리 튀는 후보 제외
        if self.prev_chain_local is not None and len(self.prev_chain_local) >= 1:
            prev_seed_x, prev_seed_y = self.prev_chain_local[0]

            seed_jump = np.sqrt(
                (x_local_all - prev_seed_x) ** 2
                + (y_local_all - prev_seed_y) ** 2
            )

            # local 좌표 jump는 회전 중 과도하게 엄격할 수 있으므로 hard reject에는 쓰지 않는다.
            # 실제 seed 연속성은 위의 이미지 좌표 seed_continuity_mask가 담당한다.
            # seed_mask = seed_mask & (seed_jump < self.seed_max_jump_px)

        # fallback 1: 기본 x/y/거리/heading 조건은 유지하되,
        # 이미지 좌표 seed continuity lock은 절대 풀지 않는다.
        if np.count_nonzero(seed_mask) == 0:
            seed_mask = (
                (np.abs(x_local_all - self.seed_waypoint_dist_px) < self.seed_band_width_px) &
                (np.abs(y_local_all) < self.seed_max_lateral_px) &
                (dist_from_robot > self.seed_min_dist_from_robot_px) &
                (dist_from_robot < self.seed_max_dist_from_robot_px) &
                (heading_dot > self.seed_min_heading_dot) &
                seed_not_visited_mask &
                seed_continuity_mask
            )

        # fallback 2: 급커브에서 seed가 안 잡힐 때만 x/heading 조건을 조금 완화.
        # 그래도 직전 accepted seed 근처라는 continuity lock은 유지한다.
        if np.count_nonzero(seed_mask) == 0:
            seed_mask = (
                (x_local_all > 15.0) &
                (x_local_all < 115.0) &
                (np.abs(y_local_all) < self.max_lateral_px) &
                (heading_dot > -0.20) &
                seed_not_visited_mask &
                seed_continuity_mask
            )

        if np.count_nonzero(seed_mask) == 0:
            return None, None, None, None

        seed_pts = pts[seed_mask]
        seed_x = x_local_all[seed_mask]
        seed_y = y_local_all[seed_mask]
        seed_dist = dist_from_robot[seed_mask]
        seed_heading_dot = heading_dot[seed_mask]
        seed_img_jump = seed_img_jump_all[seed_mask]

        seed_score = (
            0.60 * np.abs(seed_x - self.seed_waypoint_dist_px)
            + 1.20 * np.abs(seed_y)
            + 0.35 * np.abs(seed_dist - self.seed_waypoint_dist_px)
            - 20.0 * seed_heading_dot
            + self.seed_img_score_weight * seed_img_jump
        )

        if self.prev_chain_local is not None and len(self.prev_chain_local) >= 1:
            prev_x, prev_y = self.prev_chain_local[0]
            jump = np.sqrt((seed_x - prev_x) ** 2 + (seed_y - prev_y) ** 2)
            seed_score += 0.25 * jump

        seed_idx = int(np.argmin(seed_score))
        seed_pt = seed_pts[seed_idx]
        seed_local = (float(seed_x[seed_idx]), float(seed_y[seed_idx]))

        chain_img.append((int(seed_pt[0]), int(seed_pt[1])))
        chain_local.append(seed_local)

        prev_prev_pt = robot_np.copy()
        prev_pt = seed_pt.copy()

        direction = prev_pt - prev_prev_pt
        direction_norm = np.linalg.norm(direction)

        if direction_norm < 1e-6:
            direction = forward.copy()
        else:
            direction = direction / direction_norm

        # --------------------------------------------------------
        # 2) 이후 점들 한 개씩 전진
        # seed 중심 원 -> P1
        # P1 중심 원 -> P2
        # P2 중심 원 -> P3
        # --------------------------------------------------------
        for chain_idx in range(1, self.total_chain_points):
            vec_from_prev = pts - prev_pt
            dist_from_prev = np.linalg.norm(vec_from_prev, axis=1)

            ring_mask = np.abs(dist_from_prev - self.waypoint_step_px) < self.ring_band_px

            if np.count_nonzero(ring_mask) == 0:
                return None, None, None, None

            cand_pts = pts[ring_mask]
            cand_vec = vec_from_prev[ring_mask]
            cand_dist = dist_from_prev[ring_mask]

            cand_dir = cand_vec / np.maximum(cand_dist[:, None], 1e-6)
            direction_score = cand_dir @ direction

            # 뒤로 가는 후보 제거. 직각 코너는 direction_score ~= 0 이므로 살아남음.
            forward_enough_mask = direction_score > 0.05

            if np.count_nonzero(forward_enough_mask) == 0:
                return None, None, None, None

            cand_pts = cand_pts[forward_enough_mask]
            cand_dist = cand_dist[forward_enough_mask]
            direction_score = direction_score[forward_enough_mask]

            dist_to_prev_prev = np.linalg.norm(cand_pts - prev_prev_pt, axis=1)

            rel_cand = cand_pts - robot_np
            cand_x_local = rel_cand @ forward
            cand_y_local = rel_cand @ left

            not_too_back_mask = cand_x_local > -20.0

            if np.count_nonzero(not_too_back_mask) == 0:
                return None, None, None, None

            cand_pts = cand_pts[not_too_back_mask]
            cand_dist = cand_dist[not_too_back_mask]
            direction_score = direction_score[not_too_back_mask]
            dist_to_prev_prev = dist_to_prev_prev[not_too_back_mask]
            cand_x_local = cand_x_local[not_too_back_mask]
            cand_y_local = cand_y_local[not_too_back_mask]

            score = (
                1.00 * np.abs(cand_dist - self.waypoint_step_px)
                - 12.0 * direction_score
                - 0.25 * dist_to_prev_prev
                + 0.10 * np.abs(cand_y_local)
            )

            if self.prev_chain_local is not None and len(self.prev_chain_local) > chain_idx:
                prev_x, prev_y = self.prev_chain_local[chain_idx]
                jump = np.sqrt((cand_x_local - prev_x) ** 2 + (cand_y_local - prev_y) ** 2)
                score += 0.25 * jump

            best_idx = int(np.argmin(score))

            selected_pt = cand_pts[best_idx]
            selected_local = (
                float(cand_x_local[best_idx]),
                float(cand_y_local[best_idx])
            )

            chain_img.append((int(selected_pt[0]), int(selected_pt[1])))
            chain_local.append(selected_local)

            prev_prev_pt = prev_pt.copy()
            new_direction = selected_pt - prev_pt
            new_norm = np.linalg.norm(new_direction)

            if new_norm > 1e-6:
                direction = new_direction / new_norm

            prev_pt = selected_pt.copy()

        fit_chain_img = chain_img[-self.fit_waypoint_count:]
        fit_chain_local = chain_local[-self.fit_waypoint_count:]

        return chain_img, chain_local, fit_chain_img, fit_chain_local

    # ============================================================
    # HOLD recovery: rebuild shifted chain on skeleton
    # ============================================================
    def chain_min_distance_to_robot(self, chain_img, robot_center):
        """
        현재 robot_center가 chain polyline에서 얼마나 떨어져 있는지 계산한다.
        HOLD/SHIFT 중 오래된 path를 계속 물고 있지 않기 위한 안전장치.
        """
        if chain_img is None or robot_center is None or len(chain_img) < 2:
            return None

        robot_np = np.array(robot_center, dtype=np.float32)
        pts = [np.array(p, dtype=np.float32) for p in chain_img]
        min_dist = None

        for i in range(len(pts) - 1):
            a = pts[i]
            b = pts[i + 1]
            ab = b - a
            seg_len2 = float(np.dot(ab, ab))
            if seg_len2 < 1e-6:
                continue

            u = float(np.dot(robot_np - a, ab) / seg_len2)
            u = self.clamp_float(u, 0.0, 1.0)
            proj = a + u * ab
            d = float(np.linalg.norm(robot_np - proj))

            if min_dist is None or d < min_dist:
                min_dist = d

        return min_dist

    def chain_seed_distance_to_robot(self, chain_img, robot_center):
        if chain_img is None or robot_center is None or len(chain_img) < 1:
            return None
        seed_np = np.array(chain_img[0], dtype=np.float32)
        robot_np = np.array(robot_center, dtype=np.float32)
        return float(np.linalg.norm(seed_np - robot_np))

    def is_held_chain_safe_to_use(self, chain_img, robot_center):
        """
        HOLD/SHIFT에 사용할 기존 chain이 현재 robot 근처에 있는지 확인한다.
        너무 멀어진 path를 계속 제어에 사용하면 lookahead 선이 화면 반대편으로 튄다.
        """
        if chain_img is None or robot_center is None or len(chain_img) < 3:
            return False

        path_dist = self.chain_min_distance_to_robot(chain_img, robot_center)
        seed_dist = self.chain_seed_distance_to_robot(chain_img, robot_center)

        if path_dist is None or seed_dist is None:
            return False

        # 너무 작게 잡으면 코너에서 HOLD가 바로 끊기고,
        # 너무 크게 잡으면 오래된 path를 물고 간다.
        return (path_dist <= 95.0) and (seed_dist <= 140.0)

    def build_chain_from_given_seed_on_skeleton(self, skeleton, seed_img, robot_center, robot_heading_deg):
        """
        seed_img를 첫 seed로 고정하고, 이후 P1/P2/P3는 반드시 skeleton 위에서 다시 찾는다.
        SHIFT_P1에서 외삽으로 P4를 만들지 않기 위한 함수.
        반환되는 S/P1/P2/P3는 모두 skeleton point다.
        """
        ys, xs = np.where(skeleton > 0)

        if len(xs) < self.min_skeleton_pixels:
            return None, None

        pts = np.stack([xs, ys], axis=1).astype(np.float32)
        robot_np = np.array(robot_center, dtype=np.float32)
        seed_np = np.array(seed_img, dtype=np.float32)

        forward, left = self.get_basis_vectors(robot_heading_deg)

        # seed_img가 정확히 skeleton 위에 없을 수도 있으므로 가장 가까운 skeleton point로 snap.
        seed_dists = np.linalg.norm(pts - seed_np, axis=1)
        seed_idx = int(np.argmin(seed_dists))

        # 너무 멀면 old P1이 현재 skeleton 위에 없다고 판단하고 실패.
        if float(seed_dists[seed_idx]) > 18.0:
            return None, None

        seed_pt = pts[seed_idx].copy()

        # SHIFT_P1_REBUILD에서도 seed continuity를 강제한다.
        # 즉 old P1 근처 skeleton point를 seed로 쓰더라도,
        # 직전 accepted seed 기준 self.seed_img_max_jump_px 밖이면 reject한다.
        if (
            self.enable_seed_img_continuity_lock
            and self.prev_seed_img_for_lock is not None
        ):
            prev_seed_img = np.array(self.prev_seed_img_for_lock, dtype=np.float32)
            seed_jump_from_prev = float(np.linalg.norm(seed_pt - prev_seed_img))
            if seed_jump_from_prev > self.seed_img_max_jump_px:
                return None, None

        rel_seed = seed_pt - robot_np
        seed_local = (
            float(rel_seed @ forward),
            float(rel_seed @ left),
        )

        chain_img = [(int(seed_pt[0]), int(seed_pt[1]))]
        chain_local = [seed_local]

        prev_prev_pt = robot_np.copy()
        prev_pt = seed_pt.copy()

        direction = prev_pt - prev_prev_pt
        direction_norm = np.linalg.norm(direction)

        if direction_norm < 1e-6:
            direction = forward.copy()
        else:
            direction = direction / direction_norm

        for chain_idx in range(1, self.total_chain_points):
            vec_from_prev = pts - prev_pt
            dist_from_prev = np.linalg.norm(vec_from_prev, axis=1)

            ring_mask = np.abs(dist_from_prev - self.waypoint_step_px) < self.ring_band_px

            if np.count_nonzero(ring_mask) == 0:
                return None, None

            cand_pts = pts[ring_mask]
            cand_vec = vec_from_prev[ring_mask]
            cand_dist = dist_from_prev[ring_mask]

            cand_dir = cand_vec / np.maximum(cand_dist[:, None], 1e-6)
            direction_score = cand_dir @ direction

            # SHIFT 복구에서는 직각/급커브를 더 잘 따라가야 하므로 기존 0.05보다 조금 완화.
            forward_enough_mask = direction_score > -0.05

            if np.count_nonzero(forward_enough_mask) == 0:
                return None, None

            cand_pts = cand_pts[forward_enough_mask]
            cand_dist = cand_dist[forward_enough_mask]
            direction_score = direction_score[forward_enough_mask]

            dist_to_prev_prev = np.linalg.norm(cand_pts - prev_prev_pt, axis=1)

            rel_cand = cand_pts - robot_np
            cand_x_local = rel_cand @ forward
            cand_y_local = rel_cand @ left

            # 로봇 뒤쪽으로 크게 되돌아가는 후보는 제외하되, 급커브 대응을 위해 약간 완화.
            not_too_back_mask = cand_x_local > -35.0

            if np.count_nonzero(not_too_back_mask) == 0:
                return None, None

            cand_pts = cand_pts[not_too_back_mask]
            cand_dist = cand_dist[not_too_back_mask]
            direction_score = direction_score[not_too_back_mask]
            dist_to_prev_prev = dist_to_prev_prev[not_too_back_mask]
            cand_x_local = cand_x_local[not_too_back_mask]
            cand_y_local = cand_y_local[not_too_back_mask]

            score = (
                1.00 * np.abs(cand_dist - self.waypoint_step_px)
                - 12.0 * direction_score
                - 0.25 * dist_to_prev_prev
                + 0.08 * np.abs(cand_y_local)
            )

            best_idx = int(np.argmin(score))

            selected_pt = cand_pts[best_idx]
            selected_local = (
                float(cand_x_local[best_idx]),
                float(cand_y_local[best_idx])
            )

            chain_img.append((int(selected_pt[0]), int(selected_pt[1])))
            chain_local.append(selected_local)

            prev_prev_pt = prev_pt.copy()
            new_direction = selected_pt - prev_pt
            new_norm = np.linalg.norm(new_direction)

            if new_norm > 1e-6:
                direction = new_direction / new_norm

            prev_pt = selected_pt.copy()

        return chain_img, chain_local

    def rebuild_chain_from_old_p1_on_skeleton(self, skeleton, current_hold_chain, robot_center, robot_heading_deg):
        """
        HOLD가 오래 지속될 때 기존 P1을 새 seed 힌트로 사용한다.
        단, [P1, P2, P3, P4]를 외삽하지 않고, old P1 근처 skeleton point에서
        S/P1/P2/P3를 전부 다시 탐색한다.
        """
        if current_hold_chain is None or len(current_hold_chain) < 4:
            return None, None

        old_p1 = current_hold_chain[1]
        return self.build_chain_from_given_seed_on_skeleton(
            skeleton,
            old_p1,
            robot_center,
            robot_heading_deg
        )

    # ============================================================
    # Path lock
    # ============================================================
    def path_heading_from_chain(self, chain_img):
        """
        chain_img = [S, P1, P2, P3]
        P1->P2 heading을 이미지 좌표계 기준 deg로 계산.
        """
        if chain_img is None or len(chain_img) < 3:
            return None

        p1 = np.array(chain_img[1], dtype=np.float32)
        p2 = np.array(chain_img[2], dtype=np.float32)

        v = p2 - p1
        n = np.linalg.norm(v)

        if n < 1e-6:
            return None

        return self.normalize_angle_deg(math.degrees(math.atan2(v[1], v[0])))

    def should_accept_new_chain(self, new_chain_img):
        """
        새로 검출된 chain을 기존 filtered_chain_img와 비교해서
        너무 튀면 reject.
        """
        if new_chain_img is None:
            return False

        if self.filtered_chain_img is None:
            return True

        if len(new_chain_img) != len(self.filtered_chain_img):
            return False

        old_chain = [
            np.array(p, dtype=np.float32) for p in self.filtered_chain_img
        ]

        new_chain = [
            np.array(p, dtype=np.float32) for p in new_chain_img
        ]

        jumps = [
            np.linalg.norm(new_p - old_p)
            for new_p, old_p in zip(new_chain, old_chain)
        ]

        max_jump = max(jumps)

        if max_jump > self.max_chain_jump_px:
            return False

        old_heading = self.path_heading_from_chain(
            [(int(p[0]), int(p[1])) for p in old_chain]
        )
        new_heading = self.path_heading_from_chain(
            [(int(p[0]), int(p[1])) for p in new_chain]
        )

        if old_heading is not None and new_heading is not None:
            heading_jump = abs(self.normalize_angle_deg(new_heading - old_heading))

            if heading_jump > self.max_path_heading_jump_deg:
                return False

        return True

    def filter_chain_img(self, chain_img):
        """
        accepted chain만 부드럽게 갱신.
        """
        if chain_img is None:
            return None

        if self.filtered_chain_img is None:
            self.filtered_chain_img = [
                np.array(p, dtype=np.float32) for p in chain_img
            ]
            return chain_img

        if len(chain_img) != len(self.filtered_chain_img):
            self.filtered_chain_img = [
                np.array(p, dtype=np.float32) for p in chain_img
            ]
            return chain_img

        alpha = self.path_filter_alpha
        new_filtered = []

        for old_p, new_p in zip(self.filtered_chain_img, chain_img):
            new_p = np.array(new_p, dtype=np.float32)
            filtered_p = alpha * new_p + (1.0 - alpha) * old_p
            new_filtered.append(filtered_p)

        self.filtered_chain_img = new_filtered

        return [(int(p[0]), int(p[1])) for p in self.filtered_chain_img]

    # ============================================================
    # Error calculation: pure pursuit + lateral error
    # ============================================================
    def clamp_float(self, value: float, min_value: float, max_value: float) -> float:
        return max(min(value, max_value), min_value)

    def closest_point_and_lookahead_on_chain(self, chain_img, robot_center, lookahead_px):
        """
        filtered chain polyline 위에서
        1) robot_center에 가장 가까운 점
        2) 그 점부터 path를 따라 lookahead_px만큼 앞의 점
        3) lookahead 지점의 접선 벡터
        를 계산한다.
        """
        if chain_img is None or len(chain_img) < 2 or robot_center is None:
            return None

        pts = [np.array(p, dtype=np.float32) for p in chain_img]
        robot_np = np.array(robot_center, dtype=np.float32)

        best = None
        cumulative = 0.0
        segments = []

        for i in range(len(pts) - 1):
            a = pts[i]
            b = pts[i + 1]
            ab = b - a
            seg_len = float(np.linalg.norm(ab))
            if seg_len < 1e-6:
                continue

            t = float(np.dot(robot_np - a, ab) / (seg_len * seg_len))
            t = self.clamp_float(t, 0.0, 1.0)
            proj = a + t * ab
            dist = float(np.linalg.norm(robot_np - proj))
            along = cumulative + t * seg_len

            segments.append({
                "i": i,
                "a": a,
                "b": b,
                "ab": ab,
                "len": seg_len,
                "cum_start": cumulative,
                "cum_end": cumulative + seg_len,
            })

            if best is None or dist < best["dist"]:
                best = {
                    "dist": dist,
                    "proj": proj,
                    "seg_index": i,
                    "along": along,
                }

            cumulative += seg_len

        if best is None or len(segments) == 0:
            return None

        total_len = segments[-1]["cum_end"]
        target_along = min(best["along"] + lookahead_px, total_len)

        lookahead = pts[-1]
        tangent = segments[-1]["ab"] / max(segments[-1]["len"], 1e-6)

        for seg in segments:
            if seg["cum_start"] <= target_along <= seg["cum_end"]:
                u = (target_along - seg["cum_start"]) / max(seg["len"], 1e-6)
                u = self.clamp_float(float(u), 0.0, 1.0)
                lookahead = seg["a"] + u * seg["ab"]
                tangent = seg["ab"] / max(seg["len"], 1e-6)
                break

        return {
            "nearest": best["proj"],
            "lookahead": lookahead,
            "tangent": tangent,
            "cross_track_abs": best["dist"],
            "total_path_len": total_len,
        }

    def calculate_error_from_chain_img(self, chain_img, robot_center, robot_heading_deg):
        """
        기존 P2 직접 추종 대신 pure pursuit + lateral error를 사용한다.

        반환값은 debug/log 표시용 deg-equivalent error.
        실제 ESP32 제어는 /sim/path/lateral_error, /sim/path/heading_error,
        /sim/path/lookahead_error 3개 split topic만 사용한다.
        """
        if chain_img is None or len(chain_img) < 3:
            return None

        if robot_center is None or robot_heading_deg is None:
            return None

        # chain 길이가 너무 짧으면 lookahead가 과도하게 끝점으로 붙으므로 제한한다.
        lookahead_px = self.clamp_float(
            self.lookahead_distance_px,
            self.lookahead_min_px,
            self.lookahead_max_px
        )

        geom = self.closest_point_and_lookahead_on_chain(
            chain_img,
            robot_center,
            lookahead_px
        )

        if geom is None:
            return None

        robot_np = np.array(robot_center, dtype=np.float32)
        forward, left = self.get_basis_vectors(robot_heading_deg)

        lookahead = geom["lookahead"]
        nearest = geom["nearest"]
        tangent = geom["tangent"]

        rel_look = lookahead - robot_np
        target_x = float(rel_look @ forward)
        target_y = float(rel_look @ left)

        if abs(target_x) < 1.0 and abs(target_y) < 1.0:
            return None

        # Pure pursuit angle: 로봇 좌표계에서 lookahead point가 좌/우 어느 쪽인지
        lookahead_error_deg = math.degrees(
            math.atan2(target_y, max(target_x, 1.0))
        )
        lookahead_error_deg = self.normalize_angle_deg(lookahead_error_deg)

        # Signed lateral error: path tangent 기준 robot center가 어느 쪽에 있는지
        # cross(tangent, robot - nearest). 이미지 y축이 아래라 부호는 실험적으로 반대일 수 있음.
        error_vec = robot_np - nearest
        lateral_error_px = float(tangent[0] * error_vec[1] - tangent[1] * error_vec[0])

        # 위 lateral 부호가 lookahead_error와 반대로 작동하면 아래 줄에 -를 붙이면 됨.
        lateral_error_px = -lateral_error_px

        path_heading_deg = math.degrees(math.atan2(float(tangent[1]), float(tangent[0])))
        path_heading_error_deg = self.normalize_angle_deg(path_heading_deg - robot_heading_deg)

        # Smooth the three physical error components separately.
        # This keeps ESP control interpretable while avoiding frame-to-frame jitter.
        if not self.have_prev_components:
            filt_lookahead_error_deg = lookahead_error_deg
            filt_lateral_error_px = lateral_error_px
            filt_path_heading_error_deg = path_heading_error_deg
            self.have_prev_components = True
        else:
            a = self.component_smoothing_alpha
            filt_lookahead_error_deg = (
                a * lookahead_error_deg
                + (1.0 - a) * self.prev_lookahead_error_deg
            )
            filt_lateral_error_px = (
                a * lateral_error_px
                + (1.0 - a) * self.prev_lateral_error_px
            )
            filt_path_heading_error_deg = (
                a * path_heading_error_deg
                + (1.0 - a) * self.prev_path_heading_error_deg
            )

        filt_lookahead_error_deg = self.normalize_angle_deg(filt_lookahead_error_deg)
        filt_path_heading_error_deg = self.normalize_angle_deg(filt_path_heading_error_deg)

        self.prev_lookahead_error_deg = filt_lookahead_error_deg
        self.prev_lateral_error_px = filt_lateral_error_px
        self.prev_path_heading_error_deg = filt_path_heading_error_deg

        raw_control_error = (
            self.k_lookahead * filt_lookahead_error_deg
            + self.k_lateral * filt_lateral_error_px
            + self.k_heading * filt_path_heading_error_deg
        )

        raw_control_error = self.clamp_float(
            raw_control_error,
            -self.control_error_limit_deg,
            self.control_error_limit_deg
        )

        smoothed_error = (
            self.error_smoothing_alpha * raw_control_error
            + (1.0 - self.error_smoothing_alpha) * self.prev_error_deg
        )
        smoothed_error = self.normalize_angle_deg(smoothed_error)
        smoothed_error = self.clamp_float(
            smoothed_error,
            -self.control_error_limit_deg,
            self.control_error_limit_deg
        )

        self.prev_error_deg = smoothed_error

        self.last_lookahead_error_deg = filt_lookahead_error_deg
        self.last_lateral_error_px = filt_lateral_error_px
        self.last_path_heading_error_deg = filt_path_heading_error_deg
        self.last_lookahead_point = (int(lookahead[0]), int(lookahead[1]))
        self.last_nearest_point = (int(nearest[0]), int(nearest[1]))

        return smoothed_error

    def publish_split_path_errors(self):
        """Publish smoothed split errors for ESP32 control."""
        if (
            self.last_lateral_error_px is None or
            self.last_path_heading_error_deg is None or
            self.last_lookahead_error_deg is None
        ):
            return

        msg = Float32()

        msg.data = float(self.last_lateral_error_px)
        self.lateral_error_publisher.publish(msg)

        msg = Float32()
        msg.data = float(self.last_path_heading_error_deg)
        self.heading_error_publisher.publish(msg)

        msg = Float32()
        msg.data = float(self.last_lookahead_error_deg)
        self.lookahead_error_publisher.publish(msg)

    # ============================================================
    # Perception log publisher
    # ============================================================
    def publish_perception_log(
        self,
        robot_center,
        robot_heading_deg,
        filtered_chain_img,
        calculated_error
    ):
        """
        multi_logger_node가 한 줄 CSV로 합칠 수 있도록
        perception 내부 상태를 /perception/log_status 로 발행한다.

        형식 예:
        RCx:320 | RCy:240 | Head:12.5 | Spd:45.2 | W:8.1 |
        Sx:300 | Sy:210 | P1x:310 | ... | Path:ACCEPT | Err:3.2
        """
        now = self.get_clock().now()
        now_sec = now.nanoseconds * 1e-9

        speed_px_s = 0.0
        angular_w_deg_s = 0.0

        if (
            self.prev_log_time is not None and
            self.prev_robot_center_for_log is not None and
            self.prev_robot_heading_for_log is not None and
            robot_center is not None and
            robot_heading_deg is not None
        ):
            dt = now_sec - self.prev_log_time

            if dt > 1e-6:
                dx = robot_center[0] - self.prev_robot_center_for_log[0]
                dy = robot_center[1] - self.prev_robot_center_for_log[1]

                speed_px_s = math.sqrt(dx * dx + dy * dy) / dt

                d_heading = self.normalize_angle_deg(
                    robot_heading_deg - self.prev_robot_heading_for_log
                )
                angular_w_deg_s = d_heading / dt

        if robot_center is not None:
            self.prev_robot_center_for_log = robot_center

        if robot_heading_deg is not None:
            self.prev_robot_heading_for_log = robot_heading_deg

        self.prev_log_time = now_sec

        rcx = robot_center[0] if robot_center is not None else ''
        rcy = robot_center[1] if robot_center is not None else ''
        head = f'{robot_heading_deg:.3f}' if robot_heading_deg is not None else ''

        sx = sy = p1x = p1y = p2x = p2y = p3x = p3y = ''

        if filtered_chain_img is not None and len(filtered_chain_img) >= 4:
            sx, sy = filtered_chain_img[0]
            p1x, p1y = filtered_chain_img[1]
            p2x, p2y = filtered_chain_img[2]
            p3x, p3y = filtered_chain_img[3]

        err = f'{calculated_error:.3f}' if calculated_error is not None else ''
        look = f'{self.last_lookahead_error_deg:.3f}' if self.last_lookahead_error_deg is not None else ''
        lat = f'{self.last_lateral_error_px:.3f}' if self.last_lateral_error_px is not None else ''
        herr = f'{self.last_path_heading_error_deg:.3f}' if self.last_path_heading_error_deg is not None else ''
        lx = self.last_lookahead_point[0] if self.last_lookahead_point is not None else ''
        ly = self.last_lookahead_point[1] if self.last_lookahead_point is not None else ''

        log_text = (
            f'RCx:{rcx} | RCy:{rcy} | '
            f'Head:{head} | '
            f'Spd:{speed_px_s:.3f} | '
            f'W:{angular_w_deg_s:.3f} | '
            f'Sx:{sx} | Sy:{sy} | '
            f'P1x:{p1x} | P1y:{p1y} | '
            f'P2x:{p2x} | P2y:{p2y} | '
            f'P3x:{p3x} | P3y:{p3y} | '
            f'Lx:{lx} | Ly:{ly} | '
            f'LookErr:{look} | LatErr:{lat} | HeadErr:{herr} | '
            f'Path:{self.last_path_status} | '
            f'Err:{err}'
        )

        msg = String()
        msg.data = log_text
        self.log_status_publisher.publish(msg)

    # ============================================================
    # Drawing helpers
    # ============================================================
    def draw_robot_mask_debug(self, image, robot_center, robot_heading_deg):
        front_poly, body_poly = self.get_robot_mask_polygons(robot_center, robot_heading_deg)

        if front_poly is not None:
            cv2.polylines(image, [front_poly], True, (0, 0, 255), 2)

        if body_poly is not None:
            cv2.polylines(image, [body_poly], True, (0, 0, 255), 2)

    def draw_waypoint_chain(
        self,
        image,
        robot_center,
        chain_img,
        status_text=None
    ):
        """
        seed, P1, P2, P3 chain과 pure-pursuit lookahead target을 그린다.
        """
        if chain_img is None or len(chain_img) == 0:
            return

        colors = [
            (255, 255, 0),   # S
            (0, 255, 0),     # P1
            (0, 200, 255),   # P2
            (0, 128, 255),   # P3
        ]

        labels = ["S", "P1", "P2", "P3"]

        for i, pt in enumerate(chain_img):
            color = colors[min(i, len(colors) - 1)]
            label = labels[min(i, len(labels) - 1)]

            cv2.circle(image, pt, 7, color, -1)

            cv2.putText(
                image,
                label,
                (pt[0] + 6, pt[1] - 6),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                color,
                2
            )

        for i in range(len(chain_img) - 1):
            cv2.line(
                image,
                chain_img[i],
                chain_img[i + 1],
                (255, 0, 255),
                2
            )

        if self.last_nearest_point is not None:
            cv2.circle(image, self.last_nearest_point, 6, (255, 255, 255), -1)
            cv2.putText(
                image,
                "NEAR",
                (self.last_nearest_point[0] + 8, self.last_nearest_point[1] - 8),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 255, 255),
                2
            )

        if self.last_lookahead_point is not None and robot_center is not None:
            look_pt = self.last_lookahead_point

            cv2.circle(image, look_pt, 11, (255, 0, 255), -1)
            cv2.line(image, robot_center, look_pt, (255, 0, 255), 2)

            cv2.putText(
                image,
                "LOOKAHEAD",
                (look_pt[0] + 8, look_pt[1] + 8),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 0, 255),
                2
            )

        if status_text is not None and len(chain_img) >= 1:
            cv2.putText(
                image,
                status_text,
                (chain_img[0][0] + 8, chain_img[0][1] + 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 255, 255),
                2
            )

    # ============================================================
    # Main callback
    # ============================================================
    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        debug_image = cv_image.copy()
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # =====================
        # 1. Robot pose
        # =====================
        pose = self.detect_robot_pose(gray, debug_image)

        robot_center = pose["robot_center"]
        robot_heading_deg = pose["robot_heading_deg"]
        left_marker = pose["left_marker"]
        right_marker = pose["right_marker"]
        heading_end = pose["heading_end"]

        # =====================
        # 2. Search cmd
        # =====================
        search_cmd_data = self.publish_search_cmd(
            pose["marker10_seen"],
            pose["marker15_seen"]
        )

        cv2.putText(
            debug_image,
            f"SearchCmd: {search_cmd_data}",
            (20, 175),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 128, 255),
            2
        )

        # =====================
        # 3. Binary -> remove robot -> skeleton
        # =====================
        binary_raw = self.extract_line_binary(gray)

        binary_clean = self.remove_robot_from_binary(
            binary_raw,
            robot_center,
            robot_heading_deg
        )

        skeleton = self.skeletonize(binary_clean)

        self.draw_robot_mask_debug(
            debug_image,
            robot_center,
            robot_heading_deg
        )

        # =====================
        # 4. Waypoint chain with path lock
        # =====================
        candidate_chain_img = None
        chain_local = None
        fit_chain_img = None
        fit_chain_local = None
        filtered_chain_img = None
        calculated_error = None
        self.last_lookahead_error_deg = None
        self.last_lateral_error_px = None
        self.last_path_heading_error_deg = None
        self.last_lookahead_point = None
        self.last_nearest_point = None

        if robot_center is not None and robot_heading_deg is not None:
            candidate_chain_img, chain_local, fit_chain_img, fit_chain_local = self.find_waypoint_chain_from_skeleton(
                skeleton,
                robot_center,
                robot_heading_deg
            )

            accept_new_path = self.should_accept_new_chain(candidate_chain_img)

            if accept_new_path:
                filtered_chain_img = self.filter_chain_img(candidate_chain_img)
                self.path_locked = True
                self.path_hold_count = 0
                self.last_path_status = "ACCEPT"

                # accepted seed를 이미지 좌표 기준 continuity lock에 저장.
                # 이후 fallback에서도 이 seed 근처에서만 다음 seed를 찾는다.
                if candidate_chain_img is not None and len(candidate_chain_img) >= 1:
                    self.prev_seed_img_for_lock = np.array(candidate_chain_img[0], dtype=np.float32)

                # seed history blocking은 enable_seed_history_blocking=True일 때만 동작한다.
                self.update_seed_history_from_chain(candidate_chain_img, skeleton.shape)

                if chain_local is not None:
                    self.prev_chain_local = chain_local

            else:
                current_hold_chain = None
                if self.filtered_chain_img is not None:
                    current_hold_chain = [
                        (int(p[0]), int(p[1])) for p in self.filtered_chain_img
                    ]

                held_chain_safe = self.is_held_chain_safe_to_use(
                    current_hold_chain,
                    robot_center
                )

                if (
                    current_hold_chain is not None
                    and held_chain_safe
                    and self.path_hold_count < self.path_hold_max_frames
                ):
                    filtered_chain_img = current_hold_chain
                    self.path_hold_count += 1
                    self.path_locked = True
                    self.last_path_status = f"HOLD {self.path_hold_count}"

                elif current_hold_chain is not None and held_chain_safe:
                    # HOLD max 초과: RESET하지 않고 old P1을 새 seed 힌트로 하여
                    # skeleton 위에서 S/P1/P2/P3를 전부 다시 만든다.
                    rebuilt_chain_img, rebuilt_chain_local = self.rebuild_chain_from_old_p1_on_skeleton(
                        skeleton,
                        current_hold_chain,
                        robot_center,
                        robot_heading_deg
                    )

                    if (
                        rebuilt_chain_img is not None
                        and self.is_held_chain_safe_to_use(rebuilt_chain_img, robot_center)
                    ):
                        # 모든 point가 skeleton에서 다시 선택된 chain만 채택한다.
                        self.filtered_chain_img = [
                            np.array(p, dtype=np.float32) for p in rebuilt_chain_img
                        ]
                        filtered_chain_img = rebuilt_chain_img

                        self.prev_chain_local = rebuilt_chain_local
                        self.prev_seed_img_for_lock = np.array(rebuilt_chain_img[0], dtype=np.float32)

                        self.path_locked = True
                        self.path_hold_count = 0
                        self.last_path_status = "SHIFT_P1_REBUILD"
                    else:
                        self.filtered_chain_img = None
                        self.prev_chain_local = None
                        # seed continuity lock은 유지한다.
                        # RESET_STALE_SHIFT 다음 프레임에도 직전 seed 기준 30px 안에서만 재획득한다.
                        self.path_locked = False
                        self.path_hold_count = 0
                        self.last_path_status = "RESET_STALE_SHIFT"
                        filtered_chain_img = None

                else:
                    # 기존 path가 없거나 현재 robot에서 너무 멀면 stale로 판단하고 버린다.
                    self.filtered_chain_img = None
                    self.prev_chain_local = None
                    # seed continuity lock은 유지한다.
                    # RESET_STALE 이후에도 전체 skeleton에서 새로 잡지 않고,
                    # 직전 accepted seed 기준 30px 안에서만 재획득한다.
                    self.path_locked = False
                    self.path_hold_count = 0
                    self.last_path_status = "RESET_STALE"
                    filtered_chain_img = None

            if filtered_chain_img is not None:
                calculated_error = self.calculate_error_from_chain_img(
                    filtered_chain_img,
                    robot_center,
                    robot_heading_deg
                )

        # =====================
        # 5. Publish split path errors
        # =====================
        if calculated_error is not None:
            self.publish_split_path_errors()

            cv2.putText(
                debug_image,
                f"Error: {calculated_error:.2f} deg",
                (20, 70),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 0, 255),
                2
            )

            cv2.putText(
                debug_image,
                f"Target: PurePursuit+Lat / Path: {self.last_path_status}",
                (20, 105),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (255, 0, 255),
                2
            )

            self.draw_waypoint_chain(
                debug_image,
                robot_center,
                filtered_chain_img,
                self.last_path_status
            )

        else:
            cv2.putText(
                debug_image,
                "Waypoint chain not found",
                (20, 70),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 0, 255),
                2
            )

        # =====================
        # 6. Publish perception log
        # =====================
        self.publish_perception_log(
            robot_center,
            robot_heading_deg,
            filtered_chain_img,
            calculated_error
        )

        # =====================
        # 7. Publish debug image
        # =====================
        debug_msg = self.bridge.cv2_to_imgmsg(debug_image, encoding='bgr8')
        debug_msg.header.stamp = msg.header.stamp
        debug_msg.header.frame_id = msg.header.frame_id
        self.debug_image_publisher.publish(debug_msg)

        # =====================
        # 7. Skeleton debug image
        # =====================
        skeleton_vis = cv2.cvtColor(skeleton, cv2.COLOR_GRAY2BGR)

        # seed history mask 표시: 빨간색
        if self.seed_history_mask is not None:
            history_overlay = self.seed_history_mask > 0
            skeleton_vis[history_overlay] = (0, 0, 255)

        self.draw_robot_mask_debug(
            skeleton_vis,
            robot_center,
            robot_heading_deg
        )

        if left_marker is not None and right_marker is not None:
            cv2.line(skeleton_vis, tuple(left_marker), tuple(right_marker), (255, 0, 0), 2)
            cv2.circle(skeleton_vis, tuple(left_marker), 5, (0, 255, 255), -1)
            cv2.circle(skeleton_vis, tuple(right_marker), 5, (255, 255, 0), -1)

        if robot_center is not None:
            cv2.circle(skeleton_vis, robot_center, 6, (0, 0, 255), -1)

        if robot_center is not None and heading_end is not None:
            cv2.arrowedLine(skeleton_vis, robot_center, heading_end, (255, 255, 0), 2)

        if filtered_chain_img is not None:
            self.draw_waypoint_chain(
                skeleton_vis,
                robot_center,
                filtered_chain_img,
                self.last_path_status
            )

        if calculated_error is not None:
            status_text = f"Pursuit OK / Err: {calculated_error:.2f}"
            status_color = (0, 255, 0)
        else:
            status_text = "Waypoint None"
            status_color = (0, 0, 255)

        cv2.putText(
            skeleton_vis,
            status_text,
            (20, 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            status_color,
            2
        )

        cv2.putText(
            skeleton_vis,
            f"SearchCmd: {search_cmd_data}",
            (20, 70),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 128, 255),
            2
        )

        cv2.putText(
            skeleton_vis,
            f"path: {self.last_path_status} / hold: {self.path_hold_count}/{self.path_hold_max_frames}",
            (20, 105),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            2
        )

        skeleton_msg = self.bridge.cv2_to_imgmsg(skeleton_vis, encoding='bgr8')
        skeleton_msg.header.stamp = msg.header.stamp
        skeleton_msg.header.frame_id = msg.header.frame_id
        self.skeleton_debug_publisher.publish(skeleton_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info('비전 노드를 종료합니다.')

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()