import socket
import threading
import time

host='localhost'
tcp_port = 1234
udp_port = 7777

running = True
messages = []

def setup_tcp():
  tcp_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
  tcp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

  tcp_sock.bind((host, tcp_port))
  tcp_sock.listen(7)

  print(f"🚀 TCP RUNNING ON {host}:{tcp_port}")


  return tcp_sock

def setup_udp():
  udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
  udp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
  udp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

  print(f"🚀 UDP RUNNING ON {host}:{udp_port}")

  return udp_sock

def handle_client(client_sock, addr):
  global running

  msg = f"New connection from {addr[0]}"
  print(msg)
  messages.append(msg)

  while running:
    buff = client_sock.recv(1024).decode('utf-8')
    messages.append(f"[{addr[0]}]: {buff}")

  client_sock.close()

def broadcast_msg(udp_sock):
  buff = '\n'.join(messages)
  buff = buff.encode('utf-8')

  udp_sock.sendto(buff, ('192.168.1.255', udp_port))
  print("Broadcasted!")

  messages.clear()


def accept_clients(tcp_sock):
  global running

  while running:
    client_sock, addr = tcp_sock.accept()

    threading.Thread(target=handle_client, args=(client_sock, addr)).start()

def clock_pool():
  global running 
  seconds = 60

  while seconds > 0:
    print(f"[Chat time remaining]: {seconds}s")
    seconds -= 1
    time.sleep(1)

  running = False

if __name__ == "__main__":
  udp_sock = setup_udp()
  tcp_sock = setup_tcp()

  threading.Thread(target=clock_pool).start()
  threading.Thread(target=accept_clients, args=(tcp_sock,)).start()

  while running:
    try:
      udp_sock.settimeout(0.05) 
      broadcast_msg(udp_sock)
      time.sleep(0.5)

    except:
      break

  udp_sock.close()
  tcp_sock.close()