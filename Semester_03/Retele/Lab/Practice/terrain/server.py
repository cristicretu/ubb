import socket
import threading
import time

port_udp = 7777
port_tcp = 1234
host = 'localhost'
N = 5

arr = ["u"] * N

def is_explored(arr):
  return not 'u' in arr

# --------------------------

def setup_tcp():
  tcp_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
  tcp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

  tcp_sock.bind((host, port_tcp))
  tcp_sock.listen(7)

  print(f"TCP Running on {host}:{port_tcp}")

  return tcp_sock

def setup_udp():
  udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
  udp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
  udp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

  # udp_sock.bind((host, port_udp))
  print(f"UDP Running on {host}:{port_udp}")
  return udp_sock

def brodcast_arr(udp_sock):
  arr_str = ''.join(arr)
  print("Brodcasting the arr...")
  udp_sock.sendto(arr_str.encode('utf-8'), (host, port_udp))

def handle_clients(client_sock, addr):
  print(f"New connection from {addr}")

  client_sock.close()

def accept_clients(tcp_sock):
  while not is_explored(arr):
    try:
      client_sock, addr = tcp_sock.accept()
      threading.Thread(target=handle_clients, args=(client_sock, addr)).start()
    except:
      break


if __name__ == "__main__":
  udp_sock = setup_udp()
  tcp_sock = setup_tcp()

  thrd = threading.Thread(target=accept_clients, args=(tcp_sock,))
  thrd.start()

  while not is_explored(arr):
    brodcast_arr(udp_sock)
    time.sleep(2)

  udp_sock.close()
  tcp_sock.close()