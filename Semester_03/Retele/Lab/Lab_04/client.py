import socket

tcp_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
tcp_sock.connect(('127.0.0.1', 8080))
message = input("Enter a message: ")
tcp_sock.sendall(message.encode())
tcp_sock.close()
