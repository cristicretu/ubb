import socket
import threading
import random
import time

host = '172.20.10.14'
port_tcp = 1234
run_lock = threading.Lock()
running = True
words = ['parcat', 'marcat']

turn = 0
clients = []
client_socks = []
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

  resp = ''
  while running:
    data = client_sock.recv(1024)
    if not data:
        break
    
    resp = data.decode('utf-8')
    if not resp:  # Skip empty messages
        continue

    print(f"Received from {addr}: {resp}")

    if resp == word:
      client_sock.send("Y".encode('utf-8'))
      with run_lock:
        running = False
      break

    if resp == '.':
      break

    client_sock.send("N".encode('utf-8'))


    


def accept_clients(tcp_sock):
    global running
    while running:
       try:
         client_sock, addr = tcp_sock.accept()
         addr_str = f"{addr[0]}:{addr[1]}"
         clients.append(addr_str)

         client_sock.send(shuffled_word.encode('utf-8'))
         threading.Thread(target=handle_client, args=(client_sock, addr)).start()
       except:
        continue

if __name__ == "__main__":
  word = random.choice(words)
  shuffled_word = ''.join(random.sample(word, len(word)))
  print(f"Word: {word} -- Shuffled: {shuffled_word}")

  probability = random.randint(0, 10) > 6
  if probability == True:
    shuffled_word[0] = shuffled_word[0].upper()
  tcp_sock = setup_tcp()

  try:
      accept_thread = threading.Thread(target=accept_clients, args=(tcp_sock,))
      accept_thread.start()
      
      while running:
          time.sleep(0.1) 
          
  except Exception as e:
      print(f"Server error: {e}")


  tcp_sock.close()
