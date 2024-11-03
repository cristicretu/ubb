import select
import socket
import threading
import time
import random

PORT = 7777
TCP_PORT = 1234

def make_expression():
    return f"{random.randint(1, 10)}{random.choice(['+', '-', '*', '/'])}{random.randint(1, 10)}"

def handle_student(conn, addr, expressions, answers):
    try:
        print(f"Connection from {addr}")
        student_answers = []
        
        for _ in range(len(answers)):
            # Use select to check if data is available
            ready_to_read, _, _ = select.select([conn], [], [], 5)  # 5-second timeout
            if ready_to_read:
                data = conn.recv(1024).decode('utf-8').strip()
                if not data:
                    break
                
                try:
                    idx, answer = map(int, data.split('.'))
                    if 0 <= idx < len(answers):
                        student_answers.append((idx, answer))
                except ValueError:
                    print(f"Invalid data format from student: {data}")
            else:
                print(f"No data received from {addr} within timeout period")
                break
        
        score = sum(1 for idx, ans in student_answers if ans == answers[idx])
        response = f"Your score is {score} out of {len(answers)}"
        conn.send(response.encode('utf-8'))
        
    except Exception as e:
        print(f"Error handling student {addr}: {e}")
    finally:
        conn.close()

def main():
    # Create UDP broadcast socket
    broadcast_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    broadcast_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    broadcast_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    
    # Create TCP socket for student connections
    tcp_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    tcp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    tcp_sock.bind(('0.0.0.0', TCP_PORT))
    tcp_sock.listen(5)
    tcp_sock.setblocking(False)

    start_time = time.time()
    expressions = []
    answers = []
    student_threads = []

    print("Server started - broadcasting questions...")

    # Broadcast phase (12 seconds)
    while time.time() - start_time < 12:
        # Generate new expressions every 2 seconds
        expressions = [make_expression() for _ in range(3)]
        answers = [eval(expr) for expr in expressions]
        
        msg = ';'.join(expressions).encode('utf-8')
        # Send to broadcast address (192.168.1.255)
        broadcast_sock.sendto(msg, ('192.168.1.255', PORT))
        print(f"Broadcast: {msg.decode('utf-8')}")
        print(f"Answers: {answers}")

        # Check for new TCP connections
        try:
            readable, _, _ = select.select([tcp_sock], [], [], 0)
            if tcp_sock in readable:
                conn, addr = tcp_sock.accept()
                thread = threading.Thread(target=handle_student, 
                                       args=(conn, addr, expressions, answers))
                thread.start()
                student_threads.append(thread)
        except Exception as e:
            print(f"Error accepting connection: {e}")

        time.sleep(2)

    print("Broadcasting phase ended")
    broadcast_sock.close()

    # Wait for remaining students to finish (5 second timeout)
    tcp_sock.settimeout(5)
    try:
        while True:
            try:
                conn, addr = tcp_sock.accept()
                thread = threading.Thread(target=handle_student, 
                                       args=(conn, addr, expressions, answers))
                thread.start()
                student_threads.append(thread)
            except socket.timeout:
                break
    finally:
        tcp_sock.close()

    # Wait for all student threads to complete
    for thread in student_threads:
        thread.join()

    print("Server closed")

if __name__ == "__main__":
    main()