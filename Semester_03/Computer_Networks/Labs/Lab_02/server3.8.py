import socket

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind(("0.0.0.0", 5555))

s.listen()

conn, addr = s.accept()

buff_n = conn.recv(10)
n = int(buff_n.decode())
vec = []
for i in range(n):
  buff_a = conn.recv(10)
  vec.append(int(buff_a.decode()))

bigger_number = str(max(vec))

conn.send(str.encode(bigger_number.zfill(10)))
conn.close()