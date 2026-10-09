'''
F1 qualifying
Server keeps track of drivers speed, km, and laps (each client is a driver)
Server receives through UDP when the driver reaches checkpoints around the circuit: (1km), (2km), (3km)
        in pairs: (driverId, checkpointNo)
Server sends through TCP the current leaderboard (AVG. speed), km driven, laps, and if he is in the pits or not (connected or not)
Qualifying stops after 1 minute
'''
import socket
import select
import threading
import time

udp_port = 7777
tcp_port = 1234
host = 'localhost'

start_time = time.time()
clients = {} 
# structure: id: [(checkpoint, timestamp), ...] 

def running_condition():
  return time.time() - start_time < 60

def get_stats(driverId):
  if driverId not in clients:
    return None 

  total_km = len(clients[driverId])
  laps = sum([1 for x in clients[driverId] if x[0] == '0'])
  if not total_km:
    return None

  first_timestamp = clients[driverId][0][1]
  last_timestamp = clients[driverId][-1][1]
  seconds = last_timestamp - first_timestamp
  avg_speed = 3600 * total_km / seconds if seconds > 0 else 0

  return {
    'id': driverId,
    'speed': f"{avg_speed:.2f}",
    'distance': total_km,
    'laps': laps
  }

def setup_udp_sock():
  udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
  udp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)

  udp_sock.bind((host, udp_port))

  print(f"UDP Running on {host}:{udp_port}")

  return udp_sock

def setup_tcp_sock():
  tcp_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM) 
  tcp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)

  tcp_sock.bind((host, tcp_port))
  tcp_sock.listen(7)

  print(f"TCP Running on {host}:{tcp_port}")

  return tcp_sock

def tick():
  time_left = 60
  while running_condition():
    time_left -= 1
    print(f"[TIME LEFT]: {time_left}")
    time.sleep(1)

def handle_client(client_sock, addr):
  print(f"New connection from {addr}")
  try:
    while running_condition():
      for driverId in clients:
        stats = get_stats(driverId)
        if stats:
          driver_data = f"{stats['id']},{stats['speed']},{stats['distance']},{stats['laps']}\n"
          client_sock.sendall(driver_data.encode('utf-8'))
      
      client_sock.sendall(b"END\n")
      time.sleep(1)
  except (BrokenPipeError, ConnectionResetError):
    print(f"Client {addr} disconnected")
  finally:
    client_sock.close()

def client_pool(tcp_sock):
  while running_condition():
    client_sock, addr = tcp_sock.accept()

    threading.Thread(target=handle_client, args=(client_sock, addr)).start()


if __name__ == "__main__":
  udp_sock = setup_udp_sock()
  tcp_sock = setup_tcp_sock()

  thrd = threading.Thread(target=tick)
  thrd.start()

  threading.Thread(target=client_pool, args=(tcp_sock,)).start()

  while running_condition():
    received, addr = udp_sock.recvfrom(8)
    driverId, checkpoint = received.decode('utf-8').split(';')

    print(f"received: {(driverId, checkpoint)}")

    if driverId not in clients:
        clients[driverId] = []
    clients[driverId].append((checkpoint, time.time()))

    time.sleep(1)

  tcp_sock.close()
  udp_sock.close()
  exit(1)