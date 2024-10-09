import socket

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
a = input("First number: ")
b = input("Second number: ")
s.connect(("127.0.0.1", 5555))
s.send(str.encode(a))
s.send(str.encode(';'))
s.send(str.encode(b))
msg = s.recv(10)
print(msg.decode())
s.close()