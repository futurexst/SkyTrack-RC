# RC Robot Lane Tracer with ROS2 and ESP32 micro-ROS

## 1. Project Overview
카메라 기반 차선/마커 인식 결과를 ROS2 topic으로 처리하고,
ESP32 micro-ROS 펌웨어가 제어 명령을 받아 모터를 구동하는 RC 로봇 프로젝트.

## 2. System Architecture
- PC: ROS2 Jazzy, OpenCV, perception_node, logger_node
- MCU: ESP32, micro-ROS, L298N motor driver, MPU6050
- Communication: ROS2 <-> micro-ROS Agent <-> ESP32 over UDP

## 3. Main Nodes
| Node | Role |
|---|---|
| camera_node | 카메라 이미지 publish |
| perception_node | 차선/마커 인식 및 heading error 계산 |
| sim_controller_node | Gazebo 시뮬레이션 제어 |
| multi_logger_node | ESP32 debug, IMU, PWM 로그 저장 |
| video_logger_node | 주행 영상 저장 |
| ESP32 firmware | ROS2 topic subscribe 후 모터 PWM 출력 |

## 4. ROS2 Topics
| Topic | Type | Description |
|---|---|---|
| /image_raw | sensor_msgs/Image | 카메라 원본 이미지 |
| /perception/debug_image | sensor_msgs/Image | 인식 결과 디버그 이미지 |
| /target_heading_error | std_msgs/Float32 | 목표 heading error |
| /perception/search_cmd | std_msgs/Bool or Int32 | search mode 명령 |
| /esp32/debug_status | std_msgs/String | ESP32 상태 및 PWM 로그 |
| /esp32/imu_debug | std_msgs/Float32MultiArray | MPU6050 IMU 데이터 |

## 5. Requirements
- Ubuntu 24.04 / WSL2
- ROS2 Jazzy
- Python 3.12
- OpenCV
- cv_bridge
- micro-ROS Agent
- ESP-IDF v5.2

## 6. Build ROS2 Package

```bash
cd ~/Project-1_Lane_Tracer/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/local_setup.bash
