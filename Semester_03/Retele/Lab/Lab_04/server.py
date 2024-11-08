import socket
from os import fork

PORT = 8080

def handle_client(client_sock, client_addr):
    print(f"Connection from {client_addr}")
    while True:
        data = client_sock.recv(1024)
        if not data:
            break
        print(f"Received message: {data.decode()}")
    client_sock.close()
    

def accept_connection(tcp_sock):
    while True:
        client_sock, client_addr = tcp_sock.accept()
        f = fork()

        if f == 0:
            handle_client(client_sock, client_addr)
            exit()
        


tcp_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
tcp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
tcp_sock.bind(('0.0.0.0', PORT))
tcp_sock.listen(5)

accept_connection(tcp_sock)