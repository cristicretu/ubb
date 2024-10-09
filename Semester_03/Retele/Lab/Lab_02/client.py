import socket

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
msg = "sa traiti!"
s.connect(("127.0.0.1", 5555))
s.send(str.encode(msg))
msg = s.recv(10)
print(msg.decode())
s.close()