import socket

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect(("127.0.0.1", 5555))

ip = s.getsockname()[0]

s.send(ip.encode())

print("Sum of digits in my IP is ", end="")
msg = s.recv(10)
print(int(msg.decode()))

s.close()