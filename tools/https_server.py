import socket
import ssl
import threading
from http.server import HTTPServer, SimpleHTTPRequestHandler

def get_my_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))
        return s.getsockname()[0]
    finally:
        s.close()

def udp_discovery_listener():
    UDP_PORT = 19700
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('', UDP_PORT))
    print(f"[*] UDP Discovery Listener started on port {UDP_PORT}...")
    
    while True:
        data, addr = sock.recvfrom(1024)
        msg = data.decode('utf-8').strip()
        if msg == "WHO_IS_OTA_SERVER":
            my_ip = get_my_ip()
            response = f"I_AM_OTA_SERVER:{my_ip}"
            sock.sendto(response.encode('utf-8'), addr)
            print(f"    -> ESP32({addr[0]}) 요청, 내 IP({my_ip}) 응답")

def start_https_server():
    HOST = "0.0.0.0"
    PORT = 8000
    httpd = HTTPServer((HOST, PORT), SimpleHTTPRequestHandler)
    
    context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    context.load_cert_chain(certfile="server.crt", keyfile="server.key")
    httpd.socket = context.wrap_socket(httpd.socket, server_side=True)
    
    print(f"[*] Serving HTTPS OTA on port {PORT}...")
    httpd.serve_forever()

if __name__ == "__main__":
    threading.Thread(target=udp_discovery_listener, daemon=True).start()
    start_https_server()