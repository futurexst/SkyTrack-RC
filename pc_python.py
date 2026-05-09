import socket
import time

ESP32_IP = "192.168.0.9"   # 시리얼 모니터에 뜬 ESP32 IP
ESP32_PORT = 5000

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect((ESP32_IP, ESP32_PORT))

# 한 줄 읽기 함수
def recv_line(sock):
    data = b""
    while not data.endswith(b"\n"):
        chunk = sock.recv(1)
        if not chunk:
            raise ConnectionError("Disconnected")
        data += chunk
    return data.decode().strip()

# 연결 직후 첫 메시지
print("ESP32:", recv_line(sock))   # BOOT_OK 예상

# 1차 handshake
sock.sendall(b"HELLO\n")
resp = recv_line(sock)
print("ESP32:", resp)

if resp != "READY":
    raise RuntimeError("Handshake step 1 failed")

# 2차 handshake
sock.sendall(b"START\n")
resp = recv_line(sock)
print("ESP32:", resp)

if resp != "OK":
    raise RuntimeError("Handshake step 2 failed")

print("Handshake success. Receiving data...")

try:
    while True:
        line = recv_line(sock)
        print(line)
except KeyboardInterrupt:
    sock.sendall(b"STOP\n")
    print(recv_line(sock))
    sock.close()
