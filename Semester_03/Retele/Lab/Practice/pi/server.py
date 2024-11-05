import socket

host = 'localhost'
port_udp = 1234
port_tcp = 7777
running = True

approx = 0
points = []

def get_pi_approx(inside, outside):
  return 4 * inside * outside**(-1)

def get_inside_points(points):
  return sum(1 for x, y in points if x**2 + y**2 < 1)

def setup_udp_server():
  udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

  udp_sock.bind((host, port_udp))

  print(f"UDP listening on {host}:{port_udp}")
  return udp_sock

def setup_tcp_sock():
  tcp_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

  tcp_sock.bind((host, port_tcp))

  print(f"TCP server listening on {host}:{port_tcp}")
  return tcp_sock

def convert_numb(numb):
  return (numb - 50) / 50

if __name__ == "__main__":
  udp_sock = setup_udp_server()
  tcp_sock = setup_tcp_sock()

  while running:
    # receive from UDP two numbers from 0 and 100
    buff, addr = udp_sock.recvfrom(7)
    numb_str = buff.decode('utf-8')
    numb1, numb2 = numb_str.split(';')
    numb1 = ''.join(c for c in numb1 if c.isdigit())
    numb2 = ''.join(c for c in numb2 if c.isdigit())
    numb1 = convert_numb(int(numb1))
    numb2 = convert_numb(int(numb2)) 

    points.append((numb1, numb2))
    inside = get_inside_points(points)
    approx = get_pi_approx(inside, len(points))

    print(approx)
    # numb = int.from_bytes(buff, byteorder='little')
    # numb = convert_numb(numb)
    # print(numb)

  udp_sock.close()
  tcp_sock.close()