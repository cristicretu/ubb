import socket
import threading
import time

port = 1234
host = '192.168.1.140'

messages = []

def setup_tcp():
  tcp_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
  tcp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
  tcp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)

  tcp_sock.bind((host, port))
  tcp_sock.listen(5)

  print(f"[*] Listening on {host}:{port}")

  return tcp_sock

def setup_udp():
  udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
  udp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
  udp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
  udp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)

  print(f"[*] UDP Server started on {host}:{port}")

  return udp_sock

def broadcast_msg(udp_sock):
  buff = '\n'.join(messages)
  buff = buff.encode('utf-8')

  udp_sock.sendto(buff, (host, port))

  print("[*] Broadcasted!")
  messages.clear()

def handle_client(client, addr):
  msg = f"[*] {addr} Connected to server\n"
  print(msg)
  messages.append(msg)
  while True:
    data = client.recv(1024)
    if not data:
      break

    msg = f"[*] {addr} {data.decode('utf-8')}"
    print(msg)
    messages.append(msg)

def accept_clients(tcp_sock):
  while True:
    client, addr = tcp_sock.accept()
    print(f"[*] Accepted connection from {addr[0]}:{addr[1]}")

    client_handler = threading.Thread(target=handle_client, args=(client, addr))
    client_handler.start()

if __name__ == '__main__':
  tcp_sock = setup_tcp()
  udp_sock = setup_udp()
  accept_clients(tcp_sock)

  while True:
    udp_sock.settimeout(1)
    broadcast_msg(udp_sock)

    time.sleep(0.5)
    

  tcp_sock.close()