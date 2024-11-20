import socket
import time
import sys

PORT_UDP = 7777
PORT_TCP = 1234
BUFFER_SIZE = 1024

def setup_udp():
    udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    udp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    udp_sock.bind(('', PORT_UDP))
    print("Waiting for questions...")
    return udp_sock

def setup_tcp():
    tcp_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        tcp_sock.connect(('127.0.0.1', PORT_TCP))
    except socket.error as e:
        print(f"TCP connection failed: {e}")
        sys.exit(1)
    return tcp_sock

def format_answer(idx, result):
    # Format to exactly 4 characters, padding with spaces if needed
    answer = f"{idx}.{int(round(result))}"
    return answer.ljust(4)[:4]

def main():
    udp_sock = setup_udp()
    
    try:
        buffer = udp_sock.recv(BUFFER_SIZE)
    except socket.error as e:
        print(f"UDP receive failed: {e}")
        udp_sock.close()
        sys.exit(1)
        
    questions = buffer.decode('utf-8')
    print(f"Received questions: {questions}")
    udp_sock.close()

    expressions = [expr for expr in questions.split(";") if expr]
    tcp_sock = setup_tcp()
    
    for i, expr in enumerate(expressions):
        try:
            result = eval(expr)
            answer_str = format_answer(i, result)
            tcp_sock.send(answer_str.encode('utf-8'))
            time.sleep(0.1)
        except Exception as e:
            print(f"Error evaluating {expr}: {e}")
            continue

    try:
        score = tcp_sock.recv(BUFFER_SIZE).decode('utf-8')
        print(f"Score received: {score}")
    except socket.error as e:
        print(f"Failed to receive score: {e}")

    tcp_sock.close()

if __name__ == "__main__":
    main()