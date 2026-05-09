좋아. 아래는 다른 컴퓨터에서 그대로 따라 하면 ESP32 + micro-ROS + ROS2 Jazzy + UDP(Wi-Fi) 환경을 다시 재현할 수 있게 정리한 깔끔한 최종판이야.

기준 환경은 우리가 최종적으로 성공한 조합으로 맞췄다:

Ubuntu 24.04 / WSL Ubuntu 24.04
ROS2 Jazzy
ESP-IDF v5.2
ESP32
micro-ROS Agent
UDP over Wi-Fi
성공 확인 토픽: /freertos_int32_publisher
1. 전체 목표

우리가 만든 건 이 구조야:

ESP32

Wi-Fi로 공유기에 접속
micro-ROS 예제 노드 실행
Int32 메시지 publish

PC(ROS2)

micro_ros_agent 실행
ESP32가 보낸 데이터를 ROS2 토픽으로 브리지

즉 결과적으로 ROS2에서:

노드: /esp32_int32_publisher
토픽: /freertos_int32_publisher

가 보이게 만드는 과정이었어.

2. 기본 준비물

필요한 것:

ESP32 보드
USB 케이블
Ubuntu 24.04 또는 WSL Ubuntu 24.04
ROS2 Jazzy 설치 완료
같은 Wi-Fi 네트워크
ESP32가 접속할 SSID / 비밀번호
PC의 로컬 IP 주소
3. ESP-IDF 5.2 설치
3-1. 필수 패키지 설치
sudo apt update
sudo apt install -y git wget flex bison gperf python3 python3-pip python3-venv cmake ninja-build ccache libffi-dev libssl-dev dfu-util libusb-1.0-0
3-2. ESP-IDF 5.2 설치
mkdir -p ~/esp
cd ~/esp
git clone -b release/v5.2 --recursive https://github.com/espressif/esp-idf.git esp-idf-v5.2
cd ~/esp/esp-idf-v5.2
./install.sh esp32
3-3. ESP-IDF 환경 활성화

매번 ESP-IDF 작업용 터미널에서는 아래를 먼저 해야 해.

cd ~/esp/esp-idf-v5.2
. ./export.sh

확인:

idf.py --version
4. micro-ROS ESP-IDF 컴포넌트 받기
cd ~
git clone https://github.com/micro-ROS/micro_ros_espidf_component.git
cd ~/micro_ros_espidf_component
5. ESP-IDF Python 환경에 필요한 패키지 설치

이건 반드시 ESP-IDF 5.2 가상환경 pip로 설치해야 해.

/home/jsdbs/.espressif/python_env/idf5.2_py3.12_env/bin/pip install catkin_pkg lark-parser colcon-common-extensions jinja2 typeguard
/home/jsdbs/.espressif/python_env/idf5.2_py3.12_env/bin/pip uninstall -y empy
/home/jsdbs/.espressif/python_env/idf5.2_py3.12_env/bin/pip install empy==3.3.4

중요 포인트:

empy는 3.3.4
4.x 쓰면 micro-ROS 쪽에서 깨질 수 있음
6. ROS2 쪽 micro-ROS Agent 설치

이건 ROS2 터미널에서 진행한다.
ESP-IDF 터미널이랑 섞지 않는 게 중요해.

6-1. 워크스페이스 만들기
source /opt/ros/jazzy/setup.bash
mkdir -p ~/microros_ws/src
cd ~/microros_ws
git clone -b jazzy https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
6-2. 의존성 설치
sudo apt update
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -y
6-3. micro_ros_setup 빌드
cd ~/microros_ws
source /opt/ros/jazzy/setup.bash
colcon build
source install/local_setup.bash
6-4. Agent 워크스페이스 생성 및 빌드
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
source install/local_setup.bash
7. ESP32 예제 프로젝트 설정

ESP-IDF 터미널에서:

cd ~/esp/esp-idf-v5.2
. ./export.sh
cd ~/micro_ros_espidf_component/examples/int32_publisher
idf.py set-target esp32
idf.py menuconfig
8. menuconfig에서 설정할 것

메뉴에서 들어갈 곳:

micro-ROS Settings

여기서 설정:

8-1. WiFi Configuration
SSID: 공유기/핫스팟 이름
Password: Wi-Fi 비밀번호
8-2. micro-ROS Agent IP
ROS2 PC의 IP 주소
확인 명령:
hostname -I

예:

PC IP가 192.168.0.234면 그대로 넣기
8-3. micro-ROS Agent Port
8888
8-4. micro-ROS network interface
WLAN interface

나머지는 기본값 유지.

저장:

S
Enter
ESC로 종료
9. 프로젝트 빌드

ESP-IDF 터미널에서:

cd ~/esp/esp-idf-v5.2
. ./export.sh
cd ~/micro_ros_espidf_component/examples/int32_publisher
idf.py build

성공하면 마지막에 비슷하게 나와야 함:

Project build complete. To flash, run: idf.py flash
10. ESP32 플래시

포트가 /dev/ttyUSB0라고 가정하면:

idf.py -p /dev/ttyUSB0 flash

포트 확인:

ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
11. ESP32 모니터
idf.py -p /dev/ttyUSB0 monitor

정상 흐름이면:

ESP32 부팅
Wi-Fi 연결
IP 할당
publish 시작

실제로 우리가 성공했을 때는 오른쪽 모니터에 이런 식으로 떴어:

Publishing: 0
Publishing: 1
Publishing: 2
12. micro-ROS Agent 실행

이건 ROS2 터미널에서 실행:

source /opt/ros/jazzy/setup.bash
source ~/microros_ws/install/local_setup.bash
export ROS_DOMAIN_ID=0
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 -v6

이 창은 계속 켜둬야 해.

13. ROS2에서 토픽 확인

다른 ROS2 터미널에서:

source /opt/ros/jazzy/setup.bash
source ~/microros_ws/install/local_setup.bash
export ROS_DOMAIN_ID=0
ros2 node list
ros2 topic list -t

성공했을 때 실제로 보였던 것은:

노드: /esp32_int32_publisher
토픽: /freertos_int32_publisher [std_msgs/msg/Int32]

토픽 echo:

ros2 topic echo /freertos_int32_publisher
14. 성공 기준

아래가 다 만족하면 성공이야.

ESP monitor 창
Wi-Fi 연결 성공
Publishing: 0, 1, 2... 출력
Agent 창
UDP 패킷 송수신 로그가 보임
ROS2 확인 창
/esp32_int32_publisher 노드가 보임
/freertos_int32_publisher 토픽이 보임
ros2 topic echo /freertos_int32_publisher에 정수값이 나옴
15. 터미널 역할 정리

실전에서는 터미널을 3개 쓰면 편해.

터미널 A — ESP-IDF / ESP32
cd ~/esp/esp-idf-v5.2
. ./export.sh
cd ~/micro_ros_espidf_component/examples/int32_publisher
idf.py -p /dev/ttyUSB0 monitor
터미널 B — micro-ROS Agent
source /opt/ros/jazzy/setup.bash
source ~/microros_ws/install/local_setup.bash
export ROS_DOMAIN_ID=0
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 -v6
터미널 C — ROS2 확인
source /opt/ros/jazzy/setup.bash
source ~/microros_ws/install/local_setup.bash
export ROS_DOMAIN_ID=0
ros2 node list
ros2 topic list -t
ros2 topic echo /freertos_int32_publisher
16. 환경 분리 원칙

이건 진짜 중요해.

ESP-IDF 작업 터미널

여기서는:

cd ~/esp/esp-idf-v5.2
. ./export.sh
ROS2 작업 터미널

여기서는:

source /opt/ros/jazzy/setup.bash
source ~/microros_ws/install/local_setup.bash

한 터미널에서 둘 다 섞지 않는 것이 핵심이야.

17. 최종 명령어 모음
ESP-IDF 쪽
cd ~/esp/esp-idf-v5.2
. ./export.sh
cd ~/micro_ros_espidf_component/examples/int32_publisher
idf.py set-target esp32
idf.py menuconfig
idf.py build
idf.py -p /dev/ttyUSB0 flash
idf.py -p /dev/ttyUSB0 monitor
ROS2 Agent 쪽
source /opt/ros/jazzy/setup.bash
source ~/microros_ws/install/local_setup.bash
export ROS_DOMAIN_ID=0
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 -v6
ROS2 확인 쪽
source /opt/ros/jazzy/setup.bash
source ~/microros_ws/install/local_setup.bash
export ROS_DOMAIN_ID=0
ros2 node list
ros2 topic list -t
ros2 topic echo /freertos_int32_publisher
18. 한 줄 요약

우리가 최종적으로 구축한 건:

ESP32가 Wi-Fi UDP를 통해 micro-ROS Agent에 연결되고, ROS2 Jazzy 네트워크에서 /esp32_int32_publisher 노드와 /freertos_int32_publisher 토픽으로 보이도록 만든 환경

이제 다음 단계는 이 예제를 기반으로:

geometry_msgs/msg/Twist subscriber 만들기
/cmd_vel 받기
모터 PWM 제어 붙이기
엔코더/IMU publish 하기

로 넘어가면 돼.

원하면 내가 다음 답변에서 이 정리를 바탕으로 **“ESP32 + micro-ROS + /cmd_vel subscriber 최소 코드 템플릿”**까지 바로 이어서 적어줄게.
