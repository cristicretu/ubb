import socket

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind(("0.0.0.0", 5555))
s.listen()


while True:
    conn, buff = s.accept()
    print(f"received {buff}")

    ip = conn.recv(10).decode()

    sum_digits = sum(int(digit) for digit in ip if digit.isdigit())

    conn.send(str(sum_digits).encode())
    conn.close()