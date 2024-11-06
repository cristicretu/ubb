import socket
import threading
import time

port_udp = 7777
port_tcp = 1234
host = 'localhost'
N = 30

arr = ["u"] * N
arr_lock = threading.Lock() 


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
  
  print(f"UDP Running on {host}:{port_udp}")
  return udp_sock

def brodcast_arr(udp_sock):
  arr_str = ''.join(arr)
  print("Broadcasting the arr...")
  udp_sock.sendto(arr_str.encode('utf-8'), ('<broadcast>', port_udp))

def handle_clients(client_sock, addr):
    print(f"New connection from {addr}")
    while True:
        with arr_lock:
           if is_explored(arr):
              break

        try:
            buff = client_sock.recv(3)
            if not buff or len(buff) != 3: 
                break
                
            index = (buff[0] << 8) | buff[1]
            char = chr(buff[2])
            
            print(f"Received from {addr} - Index: {index}, Char: {char}")
            
            if 0 <= index < N:
                with arr_lock:  
                    arr[index] = char
        except Exception as e:
            print(f"Error handling client {addr}: {e}")
            break
    
    print(f"Client {addr} disconnected")
    client_sock.close()


def accept_clients(tcp_sock):
    while True:
        try:
            with arr_lock: 
                if is_explored(arr):
                    break
            client_sock, addr = tcp_sock.accept()
            threading.Thread(target=handle_clients, args=(client_sock, addr)).start()
        except socket.error:  
            break


if __name__ == "__main__":
    udp_sock = setup_udp()
    tcp_sock = setup_tcp()

    thrd = threading.Thread(target=accept_clients, args=(tcp_sock,))
    thrd.start()

    try:
        while not is_explored(arr):
            brodcast_arr(udp_sock)
            time.sleep(0.5)

        brodcast_arr(udp_sock) 
        tcp_sock.close()  
        thrd.join(timeout=1) 
    except KeyboardInterrupt:
        tcp_sock.close()
        thrd.join(timeout=1)
    finally:
        print("Array fully explored, shutting down...")
        udp_sock.close()