import socket
import random
import time
import os

# Constants
BOARD_SIZE = 10

# Initialize server
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
port = 6969
s.bind(('172.30.240.46', port))
s.listen(1)

print(f"Server is running on port {port} 🚀")

apple_position = [random.randint(0, BOARD_SIZE-1), random.randint(0, BOARD_SIZE-1)]

def move_apple():
    global apple_position
    print("Enter direction (w/a/s/d): ", end='', flush=True)
    direction = input().strip().lower()
    if direction == 'w':
        apple_position[1] = max(0, apple_position[1] - 1)
    elif direction == 's':
        apple_position[1] = min(BOARD_SIZE-1, apple_position[1] + 1)
    elif direction == 'a':
        apple_position[0] = max(0, apple_position[0] - 1)
    elif direction == 'd':
        apple_position[0] = min(BOARD_SIZE-1, apple_position[0] + 1)

def print_board(snake_positions):
    os.system('cls' if os.name == 'nt' else 'clear')
    for y in range(BOARD_SIZE):
        for x in range(BOARD_SIZE):
            if [x, y] == apple_position:
                print('A', end=' ')
            elif [x, y] in snake_positions:
                print('S', end=' ')
            else:
                print('.', end=' ')
        print()
    print()

client, addr = s.accept()
print(f"Connection from {addr}")

snake_positions = [[0, 0]]  

while True:
    
    client.sendall(f"{apple_position[0]},{apple_position[1]}".encode())

   
    data = client.recv(1024).decode()
    if data:
        snake_positions = [list(map(int, pos.split(','))) for pos in data.split(';')]

    move_apple()

    print_board(snake_positions)

    time.sleep(0.1)  
client.close()