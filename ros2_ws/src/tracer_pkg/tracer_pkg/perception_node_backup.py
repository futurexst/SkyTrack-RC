#!/usr/bin/env python3

import math

import cv2
import cv2.aruco as aruco
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Int32


class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')

        # =====================
        # ROS I/O
        # =====================
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10
        )

        self.error_publisher = self.create_publisher(
            Float32,
            '/target_heading_error',
            10
        )

        self.debug_image_publisher = self.create_publisher(
            Image,
            '/perception/debug_image',
            10
        )

        # 이름은 유지. 실제로는 skeleton + waypoint debug image
        self.skeleton_debug_publisher = self.create_publisher(
            Image,
            '/perception/roi_binary_image',
            10
        )

        self.search_cmd_publisher = self.create_publisher(
            Int32,
            '/perception/search_cmd',
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
        # 앞쪽 모듈 마스크: 마커/앞 바퀴/상단 구조 지우기
        self.front_mask_length_px = 60
        self.front_mask_width_px = 160
        self.front_mask_center_offset_px = 0.0

        # 뒤 body 마스크: 파란 body 지우기
        self.body_mask_length_px = 75
        self.body_mask_width_px = 110
        self.body_mask_center_offset_px = -70.0

        # =====================
        # Waypoint traversal params
        # seed -> P1 -> P2 -> P3
        # 실제 fitting/control은 P1,P2,P3 사용
        # =====================
        self.seed_waypoint_dist_px = 65.0
        self.seed_band_width_px = 22.0
        self.max_lateral_px = 180.0

        self.waypoint_step_px = 45.0
        self.ring_band_px = 12.0

        # 총 chain 개수 = seed + 미래 3점
        self.total_chain_points = 4

        # 실제 fitting에 쓸 미래 waypoint 개수
        self.fit_waypoint_count = 3

        self.min_skeleton_pixels = 20
        self.fit_lookahead_px = 120.0

        # smoothing
        self.prev_chain_local = None
        self.prev_error_deg = 0.0
        self.error_smoothing_alpha = 0.35

        self.last_fit_coeff = None
        
        # =====================
        # Seed constraints
        # =====================
        # seed가 robot_center, 즉 두 ArUco 중심점에서 너무 멀거나 가까우면 제외
        self.seed_min_dist_from_robot_px = 35.0
        self.seed_max_dist_from_robot_px = 95.0

        # seed는 차량 정면 근처에 있어야 하므로 lateral은 더 빡세게 제한
        self.seed_max_lateral_px = 90.0

        # 이전 프레임 seed와 너무 멀리 튀면 제외
        self.seed_max_jump_px = 45.0

        # seed가 차량 heading 방향과 너무 어긋나면 제외
        # 1.0: 완전 정면, 0.0: 직각, 음수: 뒤쪽
        self.seed_min_heading_dot = 0.45
        
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
            f'perception_node two-rect-mask + chain-waypoint version started. OpenCV={cv2.__version__}'
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
    # Border / robot mask
    # ============================================================

    def get_basis_vectors(self, robot_heading_deg):
        theta = math.radians(robot_heading_deg)
        forward = np.array([math.cos(theta), math.sin(theta)], dtype=np.float32)
        left = np.array([-forward[1], forward[0]], dtype=np.float32)
        return forward, left

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

        # 앞쪽 모듈 사각형
        front_center = rc_center + self.front_mask_center_offset_px * forward
        front_poly = self.make_oriented_rect(
            front_center,
            robot_heading_deg,
            self.front_mask_length_px,
            self.front_mask_width_px
        )

        # 뒤 body 사각형
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
    # Waypoint chain
    # seed -> P1 -> P2 -> P3
    # ============================================================
    def find_waypoint_chain_from_skeleton(self, skeleton, robot_center, robot_heading_deg):
        ys, xs = np.where(skeleton > 0)

        if len(xs) < self.min_skeleton_pixels:
            return None, None, None, None

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
        # 1) seed 찾기: robot_center와의 거리 제한 포함
        # --------------------------------------------------------

        # robot_center에서 각 skeleton point까지의 실제 이미지 거리
        dist_from_robot = np.linalg.norm(rel, axis=1)

        # robot_center -> skeleton point 방향 단위벡터
        dir_from_robot = rel / np.maximum(dist_from_robot[:, None], 1e-6)

        # 차량 heading 방향과 seed 후보 방향이 얼마나 비슷한지
        # 1.0: 정면, 0.0: 직각, -1.0: 후방
        heading_dot = dir_from_robot @ forward

        seed_mask = (
            # 차량 앞쪽 seed 거리 근처
            (np.abs(x_local_all - self.seed_waypoint_dist_px) < self.seed_band_width_px) &

            # 좌우로 너무 멀면 제외
            (np.abs(y_local_all) < self.seed_max_lateral_px) &

            # 마커 중심점/robot_center에서 너무 멀거나 가까우면 제외
            (dist_from_robot > self.seed_min_dist_from_robot_px) &
            (dist_from_robot < self.seed_max_dist_from_robot_px) &

            # 차량 heading 방향과 너무 어긋나는 점 제외
            (heading_dot > self.seed_min_heading_dot)
        )

        # 이전 seed가 있으면 갑자기 멀리 튀는 후보 제외
        if self.prev_chain_local is not None and len(self.prev_chain_local) >= 1:
            prev_seed_x, prev_seed_y = self.prev_chain_local[0]

            seed_jump = np.sqrt(
                (x_local_all - prev_seed_x) ** 2
                + (y_local_all - prev_seed_y) ** 2
            )

            seed_mask = seed_mask & (seed_jump < self.seed_max_jump_px)

        # fallback 1: 이전 seed jump 제한만 풀고 다시 탐색
        if np.count_nonzero(seed_mask) == 0:
            seed_mask = (
                (np.abs(x_local_all - self.seed_waypoint_dist_px) < self.seed_band_width_px) &
                (np.abs(y_local_all) < self.seed_max_lateral_px) &
                (dist_from_robot > self.seed_min_dist_from_robot_px) &
                (dist_from_robot < self.seed_max_dist_from_robot_px) &
                (heading_dot > self.seed_min_heading_dot)
            )

        # fallback 2: 급커브에서 seed가 안 잡힐 때 조금 완화
        if np.count_nonzero(seed_mask) == 0:
            seed_mask = (
                (x_local_all > 25.0) &
                (x_local_all < self.seed_max_dist_from_robot_px) &
                (np.abs(y_local_all) < self.max_lateral_px) &
                (heading_dot > 0.20)
            )

        if np.count_nonzero(seed_mask) == 0:
            return None, None, None, None

        seed_pts = pts[seed_mask]
        seed_x = x_local_all[seed_mask]
        seed_y = y_local_all[seed_mask]
        seed_dist = dist_from_robot[seed_mask]
        seed_heading_dot = heading_dot[seed_mask]

        seed_score = (
            # 목표 전방거리와 가까울수록 좋음
            0.60 * np.abs(seed_x - self.seed_waypoint_dist_px)

            # 차량 중심선에 가까울수록 좋음
            + 1.20 * np.abs(seed_y)

            # robot_center와의 실제 거리도 목표값과 가까울수록 좋음
            + 0.35 * np.abs(seed_dist - self.seed_waypoint_dist_px)

            # heading 방향과 잘 맞을수록 좋음
            - 20.0 * seed_heading_dot
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
        #     seed 중심 원 -> P1
        #     P1 중심 원 -> P2
        #     P2 중심 원 -> P3
        # --------------------------------------------------------
        for chain_idx in range(1, self.total_chain_points):
            vec_from_prev = pts - prev_pt
            dist_from_prev = np.linalg.norm(vec_from_prev, axis=1)

            # 원 둘레 근처 후보
            ring_mask = np.abs(dist_from_prev - self.waypoint_step_px) < self.ring_band_px
            if np.count_nonzero(ring_mask) == 0:
                return None, None, None, None

            cand_pts = pts[ring_mask]
            cand_vec = vec_from_prev[ring_mask]
            cand_dist = dist_from_prev[ring_mask]

            cand_dir = cand_vec / np.maximum(cand_dist[:, None], 1e-6)

            # 현재 진행 방향과의 유사도
            direction_score = cand_dir @ direction

            # 뒤로 가는 후보 제거
            # (이전 점 쪽으로 되돌아가는 후보 제거)
            forward_enough_mask = direction_score > -0.20
            if np.count_nonzero(forward_enough_mask) == 0:
                return None, None, None, None

            cand_pts = cand_pts[forward_enough_mask]
            cand_dist = cand_dist[forward_enough_mask]
            cand_dir = cand_dir[forward_enough_mask]
            direction_score = direction_score[forward_enough_mask]

            # 이전점(prev_prev_pt)과 너무 가까운 후보는 사실상 뒤로 가는 후보일 가능성 큼
            dist_to_prev_prev = np.linalg.norm(cand_pts - prev_prev_pt, axis=1)

            rel_cand = cand_pts - robot_np
            cand_x_local = rel_cand @ forward
            cand_y_local = rel_cand @ left

            # 너무 뒤쪽 후보는 버림
            not_too_back_mask = cand_x_local > -20.0
            if np.count_nonzero(not_too_back_mask) == 0:
                return None, None, None, None

            cand_pts = cand_pts[not_too_back_mask]
            cand_dist = cand_dist[not_too_back_mask]
            cand_dir = cand_dir[not_too_back_mask]
            direction_score = direction_score[not_too_back_mask]
            dist_to_prev_prev = dist_to_prev_prev[not_too_back_mask]
            cand_x_local = cand_x_local[not_too_back_mask]
            cand_y_local = cand_y_local[not_too_back_mask]

            # 점수:
            # - 반지름 step에 가까울수록 좋음
            # - 이전 진행 방향과 비슷할수록 좋음
            # - 이전이전 점에서 멀수록 좋음
            # - 이전 프레임 동일 인덱스 점과 비슷하면 좋음
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

        # 실제 fitting/control에는 미래 3점 사용
        fit_chain_img = chain_img[-self.fit_waypoint_count:]
        fit_chain_local = chain_local[-self.fit_waypoint_count:]

        return chain_img, chain_local, fit_chain_img, fit_chain_local

    # ============================================================
    # Error calculation
    # ============================================================
    def calculate_error_from_waypoints(self, fit_chain_local):
        """
        polyfit 사용하지 않음.

        fit_chain_local = [P1, P2, P3]
        이 중 P2를 직접 추적 target으로 사용.

        local coordinate:
            x: 차량 전방
            y: 차량 좌측

        heading error = atan2(P2_y, P2_x)
        """

        if fit_chain_local is None or len(fit_chain_local) < 2:
            return None, None

        # P1, P2, P3 중 P2 사용
        target_x, target_y = fit_chain_local[1]

        # target이 너무 가까우면 불안정하므로 reject
        if abs(target_x) < 1.0 and abs(target_y) < 1.0:
            return None, None

        raw_error_deg = math.degrees(
            math.atan2(target_y, max(target_x, 1.0))
        )
        raw_error_deg = self.normalize_angle_deg(raw_error_deg)

        # smoothing
        smoothed_error = (
            self.error_smoothing_alpha * raw_error_deg
            + (1.0 - self.error_smoothing_alpha) * self.prev_error_deg
        )

        smoothed_error = self.normalize_angle_deg(smoothed_error)
        self.prev_error_deg = smoothed_error

        # fit_coeff 자리에 None 반환
        return smoothed_error, None

    # ============================================================
    # Drawing helpers
    # ============================================================
    def draw_robot_mask_debug(self, image, robot_center, robot_heading_deg):
        front_poly, body_poly = self.get_robot_mask_polygons(robot_center, robot_heading_deg)

        if front_poly is not None:
            cv2.polylines(image, [front_poly], True, (0, 0, 255), 2)

        if body_poly is not None:
            cv2.polylines(image, [body_poly], True, (0, 0, 255), 2)

    def draw_waypoint_chain_and_fit(
        self,
        image,
        robot_center,
        robot_heading_deg,
        chain_img,
        fit_chain_local,
        fit_coeff
    ):
        """
        polyfit 곡선은 그리지 않음.
        seed, P1, P2, P3 chain을 그리고,
        제어 target으로 P2만 표시.
        """

        if chain_img is None or len(chain_img) == 0:
            return

        # chain_img = [seed, P1, P2, P3]
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

        # chain 연결선
        for i in range(len(chain_img) - 1):
            cv2.line(
                image,
                chain_img[i],
                chain_img[i + 1],
                (255, 0, 255),
                2
            )

        # 실제 추적 target은 P2
        # chain_img = [S, P1, P2, P3] 이므로 P2는 index 2
        if len(chain_img) >= 3 and robot_center is not None:
            p2_img = chain_img[2]

            cv2.circle(
                image,
                p2_img,
                10,
                (255, 0, 255),
                -1
            )

            cv2.line(
                image,
                robot_center,
                p2_img,
                (255, 0, 255),
                2
            )

            cv2.putText(
                image,
                "P2_TARGET",
                (p2_img[0] + 8, p2_img[1] + 8),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 0, 255),
                2
            )
        # fit coeff 있으면 곡선 표시
        if fit_coeff is not None and robot_center is not None and robot_heading_deg is not None:
            a, b, c = fit_coeff

            cx, cy = robot_center
            forward, left = self.get_basis_vectors(robot_heading_deg)

            curve_pts = []
            max_x = max(self.fit_lookahead_px, self.waypoint_step_px * self.fit_waypoint_count)

            for x_local in np.linspace(0.0, max_x, 30):
                y_local = a * x_local * x_local + b * x_local + c
                img_pt = (
                    np.array([cx, cy], dtype=np.float32)
                    + x_local * forward
                    + y_local * left
                )
                curve_pts.append((int(img_pt[0]), int(img_pt[1])))

            for i in range(len(curve_pts) - 1):
                cv2.line(image, curve_pts[i], curve_pts[i + 1], (255, 0, 255), 2)

            L = self.fit_lookahead_px
            y_L = a * L * L + b * L + c
            lookahead_img = (
                np.array([cx, cy], dtype=np.float32)
                + L * forward
                + y_L * left
            )
            lookahead_pt = (int(lookahead_img[0]), int(lookahead_img[1]))

            cv2.circle(image, lookahead_pt, 9, (255, 0, 255), -1)
            cv2.line(image, robot_center, lookahead_pt, (255, 0, 255), 2)
            cv2.putText(
                image,
                "FIT_TARGET",
                (lookahead_pt[0] + 8, lookahead_pt[1] + 8),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (255, 0, 255),
                2
            )
        else:
            # polyfit 없으면 마지막 점 방향으로만 직선 표시
            if fit_chain_local is not None and len(fit_chain_local) > 0 and robot_center is not None and robot_heading_deg is not None:
                x3, y3 = fit_chain_local[-1]
                forward, left = self.get_basis_vectors(robot_heading_deg)

                target_img = (
                    np.array(robot_center, dtype=np.float32)
                    + x3 * forward
                    + y3 * left
                )
                target_pt = (int(target_img[0]), int(target_img[1]))
                cv2.line(image, robot_center, target_pt, (255, 0, 255), 2)
                cv2.circle(image, target_pt, 8, (255, 0, 255), -1)
                cv2.putText(
                    image,
                    "DIR_TARGET",
                    (target_pt[0] + 8, target_pt[1] + 8),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (255, 0, 255),
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
        # 3. Binary -> remove border -> remove robot -> skeleton
        # =====================
        binary_raw = self.extract_line_binary(gray)
        binary_clean = self.remove_robot_from_binary(
            binary_raw,
            robot_center,
            robot_heading_deg
        )

        skeleton = self.skeletonize(binary_clean)

        # debug용 robot mask 표시
        self.draw_robot_mask_debug(
            debug_image,
            robot_center,
            robot_heading_deg
        )

        # =====================
        # 4. Waypoint chain
        # =====================
        chain_img = None
        chain_local = None
        fit_chain_img = None
        fit_chain_local = None
        calculated_error = None
        fit_coeff = None

        if robot_center is not None and robot_heading_deg is not None:
            chain_img, chain_local, fit_chain_img, fit_chain_local = self.find_waypoint_chain_from_skeleton(
                skeleton,
                robot_center,
                robot_heading_deg
            )

            if fit_chain_local is not None:
                calculated_error, fit_coeff = self.calculate_error_from_waypoints(
                    fit_chain_local
                )

                if calculated_error is not None:
                    self.prev_chain_local = chain_local
                    self.last_fit_coeff = fit_coeff

        # =====================
        # 5. Publish heading error
        # =====================
        if calculated_error is not None:
            error_msg = Float32()
            error_msg.data = float(calculated_error)
            self.error_publisher.publish(error_msg)

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
                "Target: P2 direct tracking",
                (20, 105),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (255, 0, 255),
                2
            )

            self.draw_waypoint_chain_and_fit(
                debug_image,
                robot_center,
                robot_heading_deg,
                chain_img,
                fit_chain_local,
                fit_coeff
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
        # 6. Publish debug image
        # =====================
        debug_msg = self.bridge.cv2_to_imgmsg(debug_image, encoding='bgr8')
        debug_msg.header.stamp = msg.header.stamp
        debug_msg.header.frame_id = msg.header.frame_id
        self.debug_image_publisher.publish(debug_msg)

        # =====================
        # 7. Skeleton debug image
        # =====================
        skeleton_vis = cv2.cvtColor(skeleton, cv2.COLOR_GRAY2BGR)

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

        if chain_img is not None:
            self.draw_waypoint_chain_and_fit(
                skeleton_vis,
                robot_center,
                robot_heading_deg,
                chain_img,
                fit_chain_local,
                fit_coeff
            )

        if calculated_error is not None:
            status_text = f"Waypoint OK / Err: {calculated_error:.2f}"
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
            f"step: {self.waypoint_step_px:.0f}px / ring: {self.ring_band_px:.0f}px",
            (20, 105),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            2
        )

        cv2.putText(
            skeleton_vis,
            f"front rect: {self.front_mask_length_px}x{self.front_mask_width_px}",
            (20, 135),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            2
        )

        cv2.putText(
            skeleton_vis,
            f"body rect: {self.body_mask_length_px}x{self.body_mask_width_px}",
            (20, 165),
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