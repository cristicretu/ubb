'''
Server is the auction-house, tries to sell something. Base price is a random between 50-100.
The server handles multiple clients connecting at the same time to the auction house.
Each client will send his next bid, through TCP (each bid will be higher than the current price).
The server updates the current price with the bid.
The auction ends when the server has not received a bid in the last 3 seconds, and the client with the last bid is declared a winner.
Server sends through UDP periodically the current price
'''
import socket
import threading
import random
import time

port_udp = 7777
port_tcp = 1234
host = 'localhost'

current_price =  random.randint(50, 100)
price_lock = threading.Lock()
last_update = 0

def get_running_condition():
    return last_update == 0  or time.time() - last_update < 1

def setup_tcp():
    tcp_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    tcp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    tcp_sock.bind((host, port_tcp))
    tcp_sock.listen(7)

    print(f"TCP Running on {host}:{port_tcp}")

    return tcp_sock

def setup_udp():
    udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    udp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

    print(f"UDP Running on {host}:{port_udp}")
    return udp_sock

def broadcast_price(udp_sock):
    global current_price
    price_str = str(current_price)
    udp_sock.sendto(price_str.encode('utf-8'), ('192.168.1.255', port_udp))
    print(f"[BROADCAST]: ${price_str}")


def handle_client(client_sock, addr):
    global current_price
    print(f"New connection from {addr}")
    while get_running_condition():
        buff = client_sock.recv(4).strip(b'\x00').decode('utf-8')
        buff_int = int(buff)

        with price_lock:
            if buff_int > current_price:
                current_price = buff_int
                last_update = time.time()


def clients_pool(tcp_sock):
    while get_running_condition(): 
        try: 
            client_sock, addr = tcp_sock.accept()

            threading.Thread(target=handle_client, args=(client_sock, addr)).start()
        except:
                break

if __name__ == "__main__":
     udp_sock = setup_udp()
     tcp_sock = setup_tcp()


     thrd = threading.Thread(target=clients_pool, args=(tcp_sock,))
     thrd.start()

     while get_running_condition():
        broadcast_price(udp_sock)

        time.sleep(1)

     udp_sock.close()
     tcp_sock.close()
            


