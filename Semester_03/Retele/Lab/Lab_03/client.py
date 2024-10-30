import socket

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
msg = "sa traiti!"
s.connect(("193.231.20.124", 7777))
s.send(str.encode(msg))
msg = s.recv(10)
print(msg.decode())
s.close()