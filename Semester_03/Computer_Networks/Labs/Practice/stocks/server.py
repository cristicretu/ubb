import socket
import threading
import time
import struct
import random

host = '192.168.1.138'
port_udp = 1234
port_tcp = 7777
running = True

stocks = []

def setup_udp_server():
  udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
  udp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

  udp_sock.bind((host, port_udp))

  print(f"UDP listening on {host}:{port_udp}")
  return udp_sock

def setup_tcp_sock():
  tcp_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
  tcp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1) 

  tcp_sock.bind((host, port_tcp))
  tcp_sock.listen(7)

  print(f"TCP server listening on {host}:{port_tcp}")
  return tcp_sock

def handle_client(client_sock, addr):
    global running
    while running:
      total_money = 0
      sendstr = ""

      if addr[0] not in clients:
          total_money = 0
      else:
          for stockId, quantity in clients[addr[0]]:
              price = stocks[stockId] * quantity
              total_money += price

      sendstr = f"${total_money}"
      # for stockId, quantity in clients[addr]:
      for (i,x) in enumerate(stocks):
        sendstr+=f"|{i};{x[1]}"

      client_sock.send(sendstr.encode('utf-8'))
      time.sleep(1)

      
    client_sock.close()

def accept_clients(tcp_sock):
    global running
    while running:
        try:
            client_sock, addr = tcp_sock.accept()
            threading.Thread(target=handle_client, args=(client_sock, addr)).start()
        except:
            break

def countdown():
    global running
    seconds = 60

    while seconds > 0:
        seconds-=1
        print(f"TIME remaining: {seconds}")

        time.sleep(1)

    running = False

clients = {}
# 192.168.1.1: [(stockId, quantity)]


if __name__ == "__main__":
    udp_sock = setup_udp_server()
    tcp_sock = setup_tcp_sock()

    for _ in range(5):
        price = random.randint(1, 500)
        stock = random.randint(1, 200)
        stocks.append((price, stock))

    thd = threading.Thread(target=countdown).start()

    tcp_thread = threading.Thread(target=accept_clients, args=(tcp_sock,))
    tcp_thread.start()

    while running:
        # Set a timeout on UDP socket to check running status periodically
        udp_sock.settimeout(0.1)
        try:
            buff, addr = udp_sock.recvfrom(7)
            if addr not in clients:
                clients[addr[0]] = []
            print(buff.decode('utf-8'))
        except socket.timeout:
            # Timeout occurred, check if we should continue running
            continue
        except:
            # Handle other potential errors
            break

    udp_sock.close()
    tcp_sock.close()


# numb = int.from_bytes(buff, byteorder='little')
# numb = convert_numb(numb)
# print(numb)