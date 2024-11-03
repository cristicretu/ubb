import select
import socket
import threading
import time
import random

PORT = 7777
TCP_PORT = 1234

operations = ["+", "-", "*", "/"]

def make_expression():
  return f"{random.randint(1, 100)}{random.choice(operations)}{random.randint(1, 100)}"

def handle_student(conn, addr, answers):
  print("Connection from ", addr)

  data = conn.recv(1024).decode('utf-8')
  student_answers = data.split(';')

  score = 0
  for i, ans in enumerate(student_answers.split('.')[1]):
    if int(ans) == answers[i]:
      score += 1

  conn.send(f"Your score is {score} out of {len(answers)}".encode('utf-8'))
  conn.close()

def main():
  time.sleep(5)
  broadcast_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
  broadcast_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
  broadcast_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
  broadcast_sock.bind(('0.0.0.0', PORT))

  tcp_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
  tcp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
  tcp_sock.bind(('0.0.0.0', TCP_PORT))
  tcp_sock.listen(5)

  tcp_sock.setblocking(False)

  start_time = time.time()


  expressions = []
  answers = []
  student_threads = []


  while time.time() - start_time < 12:
    current_expressions = []
    current_answers = []

    for _ in range(3):
      expression = make_expression()
      current_expressions.append(expression)
      current_answers.append(eval(expression))

    msg = ';'.join(current_expressions).encode('utf-8')
    broadcast_sock.sendto(msg, ('192.168.1.255', PORT))
    print(f"Brodcast: {msg.decode('utf-8')}")

    # store
    expressions = current_expressions
    answers = current_answers

    # check for new tcp connections
    try:
      readables, _, _ = select.select([tcp_sock], [], [], 0)
      if tcp_sock in readables:
        conn, addr = tcp_sock.accept()
        thread = threading.Thread(target=handle_student, args=(conn, addr, expressions, answers))
        thread.start()
        student_threads.append(thread)
    except:
      pass

    time.sleep(2)

  broadcast_sock.close()
  print("Broadcast closed")



  print("Waiting for student threads to finish")
  while True:
    try:
      readable, _, _ = select.select([], [], [], 5.0)
      if tcp_sock in readable:
        conn, addr = tcp_sock.accept()
        thread = threading.Thread(target=handle_student, args=(conn, addr, expressions, answers))
        thread.start()
        student_threads.append(thread)
      else:
        break
    except:
      break 

  tcp_sock.close()
  for thread in student_threads:
    thread.join()
  print("Server closed")
     

  return

main()