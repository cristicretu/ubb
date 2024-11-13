import math
import socket
import threading
import random
import time

host = '172.20.10.14'
port_tcp = 1234

alphabet="abcdefghijklmnopqrst"

def get_answer(problem):
   return eval(problem)

def setup_tcp():
  tcp_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
  tcp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

  tcp_sock.bind((host, port_tcp))
  tcp_sock.listen(7)

  print(f"TCP Running on {host}:{port_tcp}")

  return tcp_sock

def handle_client(client_sock, addr):
  print(f"New connection from: {addr}")
  trolls = 0

  while trolls < 3:
      problem = client_sock.recv(1024)
      problem = problem.decode('utf-8')

      print(f"received {problem}")
      try:
         ans = get_answer(problem)
      except:
         break

      probability = random.randint(0, 10) > 8
      if probability:
         ans = [''.join(random.choice(alphabet) for _ in range(10))]
         buff = f"{problem}={ans}".encode('utf-8')
         trolls += 1
      else:
          buff = f"{problem}={math.floor(ans)}".encode('utf-8')

      print(buff)
      client_sock.send(buff)

  client_sock.close()


def accept_clients(tcp_sock):
    while True:
       try:
         client_sock, addr = tcp_sock.accept()
         threading.Thread(target=handle_client, args=(client_sock, addr)).start()
       except:
        continue

if __name__ == "__main__":
  tcp_sock = setup_tcp()

  try:
      accept_thread = threading.Thread(target=accept_clients, args=(tcp_sock,))
      accept_thread.start()
      
      while True:
          time.sleep(0.1) 
          
  except Exception as e:
      print(f"Server error: {e}")

  tcp_sock.close()
