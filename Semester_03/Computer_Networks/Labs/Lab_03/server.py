import socket

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
port = 7777
s.bind(('172.30.240.46', port))
s.listen(5)

print(f"Server is running on port {port} 🚀")

while True:
    client, addr = s.accept()
    print(f"Connection from {addr}")
    client.sendall(b"Hello, world!")
    client.close()