import socket
import threading
import time

port_udp = 7777
port_tcp = 1234
host = '172.20.10.14'  # Your IP address
N = 10
arr = [["."] * N for _ in range(N)]
arr_lock = threading.Lock()

def is_explored(arr):
    for row in arr:
        if '.' in row:
            return False
    return True

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
    udp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
    udp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    print(f"UDP Running on {host}:{port_udp}")
    return udp_sock

def broadcast_arr(udp_sock):
    arr_str = [','.join(x) for x in arr]
    arr_str = '|'.join(arr_str)
    print("Broadcasting the arr:", arr_str)
    
    try:
        # Broadcast to all interfaces
        udp_sock.sendto(arr_str.encode('utf-8'), ('172.20.10.15', port_udp))
    except Exception as e:
        print(f"Broadcast error: {e}")

def handle_clients(client_sock, addr):
    print(f"New connection from {addr}")
    while True:
        with arr_lock:
            if is_explored(arr):
                break
        try:
            buff = client_sock.recv(5)
            if not buff or len(buff) != 5:
                break

            buff = buff.decode('utf-8')
            x, y, c = buff.split(';')
            x, y = int(x), int(y)
            
            print(f"Received from {addr}: x={x}, y={y}, c={c}")
            
            if 0 <= x < N and 0 <= y < N:
                with arr_lock:
                    arr[x][y] = c
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
            broadcast_arr(udp_sock)
            time.sleep(0.5)
        broadcast_arr(udp_sock)
        tcp_sock.close()
        thrd.join(timeout=1)
    except KeyboardInterrupt:
        tcp_sock.close()
        thrd.join(timeout=1)
    finally:
        print("Array fully explored, shutting down...")
        udp_sock.close()