import cv2
import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Int32, String
from cv_bridge import CvBridge


from flask import Flask, jsonify, render_template, request, Response
import threading
import time
import random
import requests
import psutil
from flask_sqlalchemy import SQLAlchemy
from datetime import datetime

from collections import deque
import feedparser



app = Flask(__name__, template_folder="templates", static_folder="static")

app.config['SQLALCHEMY_DATABASE_URI'] = 'mysql+pymysql://wslmaster:123@localhost/wsl_rdb'
app.config['SQLALCHEMY_TRACK_MODIFICATIONS'] = False

db = SQLAlchemy(app) # 여기서 앱을 넘겨줌




# --- [DB 테이블 구조] ---
class RobotLog(db.Model):
    __tablename__ = 'robot_logs'
    id = db.Column(db.Integer, primary_key=True)
    robot_id = db.Column(db.String(50))
    x = db.Column(db.Float)
    y = db.Column(db.Float)
    direction = db.Column(db.String(20)) # 수동 제어 방향
    floor = db.Column(db.String(10))
    battery = db.Column(db.Float)
    cpu = db.Column(db.Float)
    memory = db.Column(db.Float)
    latency_ctrl = db.Column(db.Integer)
    latency_robot = db.Column(db.Integer)
    status = db.Column(db.String(20))
    created_at = db.Column(db.DateTime, default=datetime.now)
    
    # --- 아두이노 시리얼 추가 ---
    pulse_count = db.Column(db.Integer, default=0)
    obstacle = db.Column(db.Integer, default=0)
    motor_running = db.Column(db.Integer, default=1)


# 💡 ESP32의 Wi-Fi IP 주소 입력
ESP32_IP = "http://192.168.0.10"


# --- [로봇 매니저 (Wi-Fi HTTP 통신 버전)] ---
class RobotManager:
    def __init__(self):
        self.state = {
            "robot_id": "SEONGSU_03",
            "status": "moving",
            "x": 120.5, "y": 45.2, "floor": "B1F",
            "battery_percent": 98.0,
            "battery_voltage": 49.0,
            "cpu_usage": 2.0,
            "memory_usage": 33.0,
            "elevator": 1,
            
            # 💡 [여기 추가됨!] 에러 방지를 위해 latency 데이터 명시
            "latency_total": 55,
            "latency_robot": 93,
            
            # 아두이노 센서 호환용
            "pulse_count": 0, 
            "obstacle": 0, 
            "motor": 1,
            
            # ESP32 센서 데이터
            'left_encoder': 0,
            'right_encoder': 0,
            'L_speed': 0.0,
            'R_speed': 0.0,
            'distance': 0.0,
            'speed': 0.0,
            'obstacle': 0,
            'motor': 1
        }
        self.running = True
        
        # ESP32 연결 상태 추적 (로그 스팸 방지)
        self.esp32_connected = False
        self.esp32_warned = False
        
        # 통계 추적
        self.command_count = 0
        self.last_command_time = None

        # 속도 계산을 위한 이전 데이터 저장용
        self.prev_left_enc = 0
        self.prev_right_enc = 0
        self.last_time = time.time()

        # 백그라운드에서 주기적으로 ESP32에 센서값을 물어보는 스레드
        self.thread = threading.Thread(target=self._update_data, daemon=True)
        self.thread.start()

    def _update_data(self):
        while self.running:
            current_time = time.time()
            dt = current_time - self.last_time

            # 1. ESP32에서 엔코더 값 가져오기 (HTTP GET)
            try:
                response = requests.get(f"{ESP32_IP}/getSensor", timeout=1)
                
                if response.status_code == 200:
                    data = response.text.strip().split(',')
                    if len(data) >= 2:
                        self.state["left_encoder"] = int(data[0])
                        self.state["right_encoder"] = int(data[1])
                    
                    # 처음 연결됐을 때만 한 번 로그
                    if not self.esp32_connected:
                        add_system_log(f"ESP32 연결 성공: {ESP32_IP}", "success")
                        self.esp32_connected = True
                        self.esp32_warned = False
                    
                    # 5초마다 상태 보고
                    if int(current_time) % 5 == 0: 
                        add_system_log(f"Topic: /getSensor | L:{self.state['left_encoder']} R:{self.state['right_encoder']}", "info")    
                else:
                    if not self.esp32_warned:
                        add_system_log(f"ESP32 응답 이상 (HTTP {response.status_code})", "warn")
                        self.esp32_warned = True
                    self.esp32_connected = False

            except Exception as e:
                # 처음 끊겼을 때만 한 번 경고, 이후로는 조용히 재시도
                if not self.esp32_warned:
                    add_system_log(f"ESP32 미연결 ({ESP32_IP}) - 재연결 시도 중...", "warn")
                    self.esp32_warned = True
                self.esp32_connected = False
                time.sleep(1.0)

            # 2. 양쪽 바퀴 속도 및 이동거리 계산
            if dt > 0.1: # 0.1초 이상 지났을 때만 계산
                meters_per_tick = 0.005 # (주의: RC카 사양에 맞게 수정!)
                
                dl = self.state["left_encoder"] - self.prev_left_enc
                dr = self.state["right_encoder"] - self.prev_right_enc
                
                self.state["L_speed"] = round((dl * meters_per_tick) / dt, 2)
                self.state["R_speed"] = round((dr * meters_per_tick) / dt, 2)
                self.state["speed"] = round((self.state["L_speed"] + self.state["R_speed"]) / 2, 2)
                
                # 누적 이동 거리 
                self.state["distance"] += round(((dl + dr) / 2.0) * meters_per_tick, 3)

                self.prev_left_enc = self.state["left_encoder"]
                self.prev_right_enc = self.state["right_encoder"]
                self.last_time = current_time

            # 가짜 배터리/시스템 데이터 시뮬레이션
            if self.state["battery_percent"] > 0:
                self.state["battery_percent"] -= random.uniform(0.01, 0.05)
            self.state["cpu_usage"] = round(random.uniform(1.0, 15.0), 1)
            
            # 너무 빨리 요청해서 ESP32가 뻗지 않도록 0.2초 딜레이
            time.sleep(0.2)

    def get_state(self):
        state_copy = self.state.copy()
        state_copy["battery_percent"] = round(state_copy["battery_percent"], 1)
        return state_copy

    # 💡 [핵심] 대시보드에서 누른 방향키를 ESP32 IP로 HTTP GET 전송!
    def send_motor_command(self, cmd):
            self.command_count += 1
            self.last_command_time = time.time()
            try:
                requests.get(f"{ESP32_IP}/{cmd}", timeout=1)
                print(f"📡 무선 명령 전송 성공: {cmd}")
            except Exception as e:
                print(f"❌ 무선 명령 전송 실패: {e}")

robot_manager = RobotManager()


# “가장 최근 카메라 화면 1장” 저장하는 변수 latest_frame = None
latest_frame = None

class RosImageSubscriber(Node):
    def __init__(self):
        super().__init__('flask_image_subscriber')
        self.bridge = CvBridge()
        self.frame_count = 0
        self.first_frame_logged = False
        self.last_frame_time = time.time()

        self.subscription = self.create_subscription(
            Image,
            '/real/perception/debug_image',
            self.listener_callback,
            qos_profile_sensor_data
        )

        self.get_logger().info('Subscribed to /real/perception/debug_image')
        add_system_log("ROS subscriber 시작: /real/perception/debug_image", "ros")

    def listener_callback(self, msg):
        global latest_frame
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            latest_frame = frame
            self.frame_count += 1
            self.last_frame_time = time.time()
            
            # 첫 프레임 수신 로그
            if not self.first_frame_logged:
                add_system_log("실제 카메라 스트림 수신 시작", "success")
                self.first_frame_logged = True
            
            # 1000프레임마다 통계
            if self.frame_count % 1000 == 0:
                add_system_log(f"카메라 프레임 누적 수신: {self.frame_count}장", "ros")
                
        except Exception as e:
            self.get_logger().error(f'Image convert error: {e}')
            add_system_log(f"카메라 변환 오류: {str(e)}", "warn")
            
            
# 속도/거리/heading 구독
latest_speed = 0.0
latest_distance = 0.0
latest_heading = 0.0

# ====== ESP32 String 토픽에서 파싱한 값 ======
# /esp32/status (시스템 진단)
esp_loop_rate   = 0.0   # Hz
esp_loop_dt     = 0.0   # ms
esp_max_dt      = 0.0   # ms
esp_wifi        = 0
esp_rssi        = 0     # dBm
esp_ch          = 0
esp_ip          = ""
esp_free_heap   = 0     # bytes
esp_min_heap    = 0     # bytes

# /esp32/debug_status (path + PWM)
latest_lateral_error = 0.0
latest_heading_error = 0.0
latest_lookahead_error = 0.0
latest_pwm_l = 0
latest_pwm_r = 0
latest_steer = 0.0
latest_hprio = 0.0
latest_search_cmd = 0   # /real/perception/search_cmd 유지

# /esp32/imu_debug — gyro_x 적분 → heading
imu_gyro_x      = 0.0   # 현재 각속도 [°/s]
heading_theta   = 0.0   # 적분된 누적 헤딩 [°]
_imu_last_t     = None  # 적분용 시각

# /real/perception/log_status — ArUco 마커 중심
marker_cx       = 0.0   # px (RCx)
marker_cy       = 0.0   # px (RCy)
marker_head     = 0.0   # ° (Head, perception 추정)

# 좌표 변환 행렬 결과 (B-1: 입력 (0,0))
transform_xc    = 0.0
transform_yc    = 0.0
transform_s     = 4.27  # cm -> px 환산 상수


def _parse_pipe_kv(s, separator='|'):
    """ 'A:1 | B:2.3 | C:4' → {'A':'1', 'B':'2.3', 'C':'4'} """
    out = {}
    if not s:
        return out
    for chunk in s.split(separator):
        if ':' not in chunk:
            continue
        k, _, v = chunk.partition(':')
        out[k.strip()] = v.strip()
    return out


def _safe_float(s, default=0.0):
    try:
        return float(s) if s not in (None, '') else default
    except (ValueError, TypeError):
        return default


def _safe_int(s, default=0):
    try:
        return int(float(s)) if s not in (None, '') else default
    except (ValueError, TypeError):
        return default


class RosStatsSubscriber(Node):
    def __init__(self):
        super().__init__('flask_stats_subscriber')

        # ESP32 시스템 상태
        self.create_subscription(String, '/esp32/status',
            self.esp_status_callback, qos_profile_sensor_data)

        # ESP32 path + PWM
        self.create_subscription(String, '/esp32/debug_status',
            self.esp_debug_callback, qos_profile_sensor_data)

        # ESP32 IMU
        self.create_subscription(String, '/esp32/imu_debug',
            self.esp_imu_callback, qos_profile_sensor_data)

        # Perception 마커 중심
        self.create_subscription(String, '/real/perception/log_status',
            self.perception_log_callback, qos_profile_sensor_data)

        # search_cmd 유지
        self.create_subscription(Int32, '/real/perception/search_cmd',
            self.search_cmd_callback, qos_profile_sensor_data)

        add_system_log("ROS subscriber 시작: ESP32 통합 + perception", "ros")

    # ---------- /esp32/status ----------
    def esp_status_callback(self, msg):
        global esp_loop_rate, esp_loop_dt, esp_max_dt, esp_wifi, esp_rssi
        global esp_ch, esp_ip, esp_free_heap, esp_min_heap
        kv = _parse_pipe_kv(msg.data)
        esp_loop_rate = _safe_float(kv.get('LoopRate'))
        esp_loop_dt   = _safe_float(kv.get('LoopDt'))
        esp_max_dt    = _safe_float(kv.get('MaxLoopDt'))
        esp_wifi      = _safe_int(kv.get('WiFi'))
        esp_rssi      = _safe_int(kv.get('RSSI'))
        esp_ch        = _safe_int(kv.get('CH'))
        esp_ip        = kv.get('IP', '')
        esp_free_heap = _safe_int(kv.get('FreeHeap'))
        esp_min_heap  = _safe_int(kv.get('MinFreeHeap'))

    # ---------- /esp32/debug_status ----------
    def esp_debug_callback(self, msg):
        global latest_lateral_error, latest_heading_error, latest_lookahead_error
        global latest_pwm_l, latest_pwm_r, latest_steer, latest_hprio
        kv = _parse_pipe_kv(msg.data)
        latest_lateral_error   = _safe_float(kv.get('Lat'))
        latest_heading_error   = _safe_float(kv.get('Head'))
        latest_lookahead_error = _safe_float(kv.get('Look'))
        latest_pwm_l           = _safe_int(kv.get('L'))
        latest_pwm_r           = _safe_int(kv.get('R'))
        latest_steer           = _safe_float(kv.get('Steer'))
        latest_hprio           = _safe_float(kv.get('HPrio'))

    # ---------- /esp32/imu_debug ----------
    def esp_imu_callback(self, msg):
        global imu_gyro_x, heading_theta, _imu_last_t
        kv = _parse_pipe_kv(msg.data)
        imu_gyro_x = _safe_float(kv.get('Gx'))
        # gyro_x 적분 → heading [°]
        now = time.time()
        if _imu_last_t is not None:
            dt = now - _imu_last_t
            if 0 < dt < 1.0:   # 비정상 dt 무시
                heading_theta += imu_gyro_x * dt
                # -180 ~ +180 정규화
                heading_theta = ((heading_theta + 180.0) % 360.0) - 180.0
        _imu_last_t = now

    # ---------- /real/perception/log_status ----------
    def perception_log_callback(self, msg):
        global marker_cx, marker_cy, marker_head, transform_xc, transform_yc
        kv = _parse_pipe_kv(msg.data)
        marker_cx   = _safe_float(kv.get('RCx'))
        marker_cy   = _safe_float(kv.get('RCy'))
        marker_head = _safe_float(kv.get('Head'))
        # B-1: 입력 (x_R, y_R) = (0, 0) 변환 결과 (= 마커가 카메라좌표에서 어디로 매핑되는지)
        # [x_C; y_C] = (1/s) * R(θ) * [-cx; -cy]
        import math
        th = math.radians(heading_theta)
        cos_t, sin_t = math.cos(th), math.sin(th)
        dx, dy = -marker_cx, -marker_cy
        transform_xc = (cos_t * dx + sin_t * dy) / transform_s
        transform_yc = (-sin_t * dx + cos_t * dy) / transform_s

    # ---------- /real/perception/search_cmd ----------
    def search_cmd_callback(self, msg):
        global latest_search_cmd
        latest_search_cmd = int(msg.data)
            
            
            
# Flask랑 ROS를 같이 돌리려고 별도 스레드에서 ROS를 실행하는 함수
def ros_spin():
    print('[ROS] ros_spin started')
    add_system_log("ROS 노드 초기화 중...", "ros")
    rclpy.init()
    
    executor = rclpy.executors.MultiThreadedExecutor()
    image_node = RosImageSubscriber()
    stats_node = RosStatsSubscriber()
    executor.add_node(image_node)
    executor.add_node(stats_node)
    
    try:
        executor.spin()
    finally:
        image_node.destroy_node()
        stats_node.destroy_node()
        rclpy.shutdown()


# OpenCV 이미지를 JPG로 바꿔서 브라우저가 볼 수 있게 계속 뿌리는 부분. 
# Flask에서는 이런 MJPEG 형식을 자주 씀    
def generate_frames():
    global latest_frame

    while True:
        if latest_frame is None:
            time.sleep(0.05)
            continue

        ret, buffer = cv2.imencode('.jpg', latest_frame)
        if not ret:
            continue

        frame_bytes = buffer.tobytes()

        yield (
            b'--frame\r\n'
            b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n'
        )

# 최신 로그 20개를 저장할 메모리 큐
system_logs = deque(maxlen=20)

def add_system_log(msg, log_type="info"):
    """시스템 로그 큐에 메시지 추가"""
    timestamp = datetime.now().strftime("%H:%M:%S")
    system_logs.append({"time": timestamp, "msg": msg, "type": log_type})

# =====================
# 뉴스 RSS (AI타임스 + 로봇신문)
# =====================
NEWS_FEEDS = [
    # 정상 작동 확인 (2026.05.04 기준)
    ("AI타임스", "https://www.aitimes.com/rss/allArticle.xml"),
    ("로봇신문", "http://www.irobotnews.com/rss/allArticle.xml"),
    
    # 전자신문 최신 RSS 주소로 교체 (정상 작동 확인)
    ("전자신문 - AI", "http://rss.etnews.com/04046.xml"),
    ("전자신문 - 로봇", "http://rss.etnews.com/06065.xml"),
    ("전자신문 - 모빌리티", "http://rss.etnews.com/17.xml"),
]

news_cache = []

def fetch_news_thread():
    """RSS 피드를 5분마다 가져와서 캐시. 누적 500건마다 로그 1번"""
    global news_cache
    cumulative = 0
    log_step = 500
    next_log_threshold = log_step

    while True:
        items = []
        for source, url in NEWS_FEEDS:
            try:
                feed = feedparser.parse(url)
                for entry in feed.entries[:15]:
                    items.append({
                        "source": source,
                        "title": entry.get("title", "").strip(),
                        "link": entry.get("link", "")
                    })
            except Exception as e:
                add_system_log(f"RSS 가져오기 실패 ({source}): {e}", "warn")

        if items:
            news_cache = items
            cumulative += len(items)

            # 누적 500건 단위로만 로그
            if cumulative >= next_log_threshold:
                add_system_log(f"뉴스 갱신 완료 {cumulative}건", "info")
                next_log_threshold += log_step

        time.sleep(300)  # 5분
    
def system_monitor_thread():
    """주기적으로 시스템 상태 로그"""
    while True:
        time.sleep(30)  # 30초마다
        try:
            cpu = psutil.cpu_percent(interval=1)
            mem = psutil.virtual_memory().percent
            add_system_log(f"시스템 상태 | CPU: {cpu}% | MEM: {mem}%", "system")
            
            # 명령 통계
            if robot_manager.command_count > 0:
                add_system_log(f"누적 원격 명령: {robot_manager.command_count}회", "system")
        except Exception as e:
            pass

# --- [API 라우터] ---
@app.route('/')
def dashboard():
    return render_template('index.html')

# 웹캠 /video_feed 라우트
@app.route('/video_feed')
def video_feed():
    return Response(
        generate_frames(),
        mimetype='multipart/x-mixed-replace; boundary=frame'
    )

@app.route('/api/robot_stats')
def get_robot_stats():
    current_state = robot_manager.get_state()

    # DB에 실시간 상태 기록
    new_log = RobotLog(
        robot_id=current_state['robot_id'],
        x=current_state['x'],
        y=current_state['y'],
        floor=current_state['floor'],
        battery=current_state['battery_percent'],
        cpu=current_state['cpu_usage'],
        memory=current_state['memory_usage'],
        latency_ctrl=current_state['latency_total'],
        latency_robot=current_state['latency_robot'],
        status=current_state['status'],
        
        pulse_count=current_state['pulse_count'],
        obstacle=current_state['obstacle'],
        motor_running=current_state['motor']
    )
    db.session.add(new_log)
    db.session.commit()

    return jsonify({
        "status": "success",
        "x": current_state['x'],
        "y": current_state['y'],
        "floor": current_state['floor'],
        "elevator": current_state['elevator'],

        # ── 속도/거리: PWM 기반 추정 (단순 스케일) ─────────
        # PWM L,R (0~255 가정) → 0~1.0 m/s 매핑
        'L_speed': round(latest_pwm_l / 255.0, 3),
        'R_speed': round(latest_pwm_r / 255.0, 3),
        'speed':   round((latest_pwm_l + latest_pwm_r) / 510.0, 3),

        # ── path 에러 (debug_status에서 파싱) ─────────────
        'lateral_error': latest_lateral_error,
        'heading_error': latest_heading_error,
        'lookahead_error': latest_lookahead_error,
        'steer': latest_steer,
        'hprio': latest_hprio,
        'pwm_l': latest_pwm_l,
        'pwm_r': latest_pwm_r,
        'search_cmd': latest_search_cmd,

        # ── IMU (heading 적분값) ──────────────────────────
        'heading': round(heading_theta, 2),
        'gyro_x':  round(imu_gyro_x, 3),

        # ── ESP32 시스템 진단 (HLTH_002 새 라벨) ──────────
        'esp': {
            'loop_rate': esp_loop_rate,
            'loop_dt':   esp_loop_dt,
            'max_dt':    esp_max_dt,
            'wifi':      esp_wifi,
            'rssi':      esp_rssi,
            'ch':        esp_ch,
            'ip':        esp_ip,
            'free_heap': esp_free_heap,
            'min_heap':  esp_min_heap
        },

        # ── 좌표 변환 (MAP_005 행렬 + 결과) ────────────────
        'transform': {
            'theta_deg': round(heading_theta, 2),
            'cx': marker_cx,
            'cy': marker_cy,
            's':  transform_s,
            'xc': round(transform_xc, 2),
            'yc': round(transform_yc, 2)
        },

        # ── (호환성) zone2 — 기존 카드가 참조 시 대비 ─────
        "zone2": {
            "on_count": 14,
            "off_count": 0,
            "battery_percent": current_state['battery_percent'],
            "voltage": current_state['battery_voltage'],
            "cpu_usage": current_state['cpu_usage'],
            "memory_usage": current_state['memory_usage']
        },
        "zone4": {
            "control_latency": current_state['latency_total'],
            "robot_latency": current_state['latency_robot']
        },

        "pulse": current_state['pulse_count'],
        "obstacle": "감지됨 ⚠️" if current_state['obstacle'] == 1 else "안전",
        "motor": "정지 ⛔" if current_state['motor'] == 0 else "가동중 🟢"
    })

@app.route('/update_position', methods=['POST'])
def update_position():
    data = request.get_json()
    if not data:
        return jsonify({"status": "error", "message": "데이터가 없습니다."}), 400

    x = data.get('x', 0)
    y = data.get('y', 0)
    direction = data.get('direction', 'S')
    
    # 1. 무선으로 ESP32에 명령 전송
    robot_manager.send_motor_command(direction)
    
    # 2. 실시간 시스템 로그 큐에 추가 (return 하기 전에 실행!)
    add_system_log(f"원격 명령 전송: {direction}", "tx")

    try:
        # 3. DB 기록[cite: 4]
        new_control_log = RobotLog(
            robot_id="SEONGSU_03",
            x=float(x),
            y=float(y),
            direction=direction,
            status="manual_control",
            floor="B1F"
        )
        db.session.add(new_control_log)
        db.session.commit()
        
        robot_manager.state["x"] = float(x)
        robot_manager.state["y"] = float(y)
        
        return jsonify({"status": "success"}) # 함수 종료[cite: 4]
    
    except Exception as e:
        db.session.rollback()
        # 에러 발생 시에도 로그를 남겨주면 디버깅에 좋습니다.
        add_system_log(f"명령 전송 에러: {str(e)}", "warn")
        print(f"Update Position Error: {e}")
        return jsonify({"status": "error", "message": str(e)}), 500
    
@app.route('/api/weather')
def get_weather():
    try:
        lat = 37.5636
        lon = 126.9976

        url = (
            "https://api.open-meteo.com/v1/forecast"
            f"?latitude={lat}&longitude={lon}"
            "&current=temperature_2m,relative_humidity_2m,weather_code"
            "&timezone=Asia%2FSeoul"
        )

        response = requests.get(url, timeout=3)
        response.raise_for_status()
        data = response.json()

        current = data.get("current", {})
        weather_code = current.get("weather_code", -1)

        weather_text_map = {
            0: "맑음",
            1: "대체로 맑음",
            2: "구름 조금",
            3: "흐림",
            45: "안개",
            48: "서리 안개",
            51: "이슬비",
            53: "보통 이슬비",
            55: "강한 이슬비",
            61: "약한 비",
            63: "보통 비",
            65: "강한 비",
            71: "약한 눈",
            73: "보통 눈",
            75: "강한 눈",
            80: "소나기",
            81: "강한 소나기",
            82: "매우 강한 소나기",
            95: "뇌우"
        }

        weather_text = weather_text_map.get(weather_code, "날씨 정보")

        return jsonify({
            "status": "success",
            "summary": weather_text,
            "temp": f"{current.get('temperature_2m', '--')}°C",
            "humidity": f"{current.get('relative_humidity_2m', '--')}%"
        })

    except Exception as e:
        return jsonify({
            "status": "error",
            "summary": "날씨 오류",
            "temp": "--°C",
            "humidity": "--%",
            "message": str(e)
        }), 500
        
# --- [API 라우터 추가] ---
@app.route('/api/system_logs')
def get_system_logs():
    return jsonify(list(system_logs))

@app.route('/api/news')
def get_news():
    return jsonify(news_cache)





if __name__ == '__main__':
    add_system_log("Flask 서버 시작 중...", "system")
    
    with app.app_context():
        db.create_all()
        add_system_log("DB 연결 성공: wsl_rdb", "success")

    ros_thread = threading.Thread(target=ros_spin, daemon=True)
    ros_thread.start()
    
    monitor_thread = threading.Thread(target=system_monitor_thread, daemon=True)
    monitor_thread.start()
    add_system_log("시스템 모니터 스레드 시작", "system")
    
    news_thread = threading.Thread(target=fetch_news_thread, daemon=True)
    news_thread.start()
    add_system_log("뉴스 RSS 스레드 시작", "system")
    
    add_system_log("대시보드 준비 완료 - http://localhost:5000", "success")
    
    app.run(host='0.0.0.0', port=5000, debug=True, use_reloader=False)