import socket

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect(("127.0.0.1", 5555))
print("no. of elements: ")
n = int(input())
s.send(str.encode(str(n).zfill(10)))
vec = []
for i in range(n):
  a = input(">")
  vec.append(a)

for a in vec:
  s.send(str.encode(str(a).zfill(10)))

print("Biggest number is: ")
msg = s.recv(10)
print(int(msg.decode()))
s.close()