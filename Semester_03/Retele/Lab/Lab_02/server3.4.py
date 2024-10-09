import socket

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind(("0.0.0.0", 5555))

s.listen()

conn, addr = s.accept()

buff = conn.recv(10)

buff_a = buff.decode().split(';')[0]
buff_b = buff.decode().split(';')[1]

bigger_number = max(buff_a, buff_b)

conn.send(str.encode(bigger_number))
