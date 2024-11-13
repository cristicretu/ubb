import socket
import threading
import random

host = '172.20.10.14'
port_tcp = 1234
running = True
my_number = random.randint(1, 100)

turn = 0
clients = []
ln = 0

def switch(chr):
  return 'H' if chr == 'L' else 'L'

def setup_tcp():
  tcp_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
  tcp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

  tcp_sock.bind((host, port_tcp))
  tcp_sock.listen(7)

  print(f"TCP Running on {host}:{port_tcp}")

  return tcp_sock

def handle_client(client_sock, addr):
  print(f"New connection from: {addr}")
  global running, turn
  my_turn = -1

  for i, client in enumerate(clients):
    if client[0] == f"{addr[0]}:{addr[1]}":
      my_turn = i
      break

  while running:
    buff = client_sock.recv(2)

    try:
      buff = int(buff.decode('utf-8'))
    except:
      continue

    while my_turn != turn:
      pass

    print(f"received {buff}")
    incorrect = random.randint(0, 10) > 7 
    ans = 'D'

    if buff != my_number:
       if buff > my_number:
          ans = 'H'

          if incorrect:
            ans = switch(ans)
       else:
          ans = 'L'

          if incorrect:
            ans = switch(ans)

    print(f"Sending:.... {ans}")
    client_sock.send(ans.encode('utf-8'))



def accept_clients(tcp_sock):
    global running
    while running:
      client_sock, addr = tcp_sock.accept()
      addr_str = f"{addr[0]}:{addr[1]}"
      clients.append(addr_str)

      threading.Thread(target=handle_client, args=(client_sock, addr)).start()

if __name__ == "__main__":
  print(f"MY NUMBER IZZ {my_number}")
  tcp_sock = setup_tcp()

  thrd = threading.Thread(target=accept_clients, args=(tcp_sock,))
  thrd.start()

  while running:
    pass


  thrd.join()
  tcp_sock.close()
