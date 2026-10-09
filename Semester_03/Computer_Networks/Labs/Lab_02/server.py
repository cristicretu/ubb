import socket

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind(("0.0.0.0", 5555))

s.listen()

conn, addr = s.accept()

buff = conn.recv(10)
print(buff.decode())

conn.send(str.encode("TE SALUT!"))
