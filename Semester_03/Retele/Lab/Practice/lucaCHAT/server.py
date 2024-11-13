import socket
import threading
import time
import json
from queue import Queue, Empty  

port_tcp = 1234
host = '172.20.10.14'
running = True
clients = {}
messages_lock = threading.Lock()

def setup_tcp():
    tcp_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    tcp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    tcp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
    tcp_sock.bind((host, port_tcp))
    tcp_sock.listen(7)
    print(f"Listening on {host}:{port_tcp}")
    return tcp_sock

def broadcast_message(sender_addr, message):
    with messages_lock:
        for addr, client_info in clients.items():
            if addr != sender_addr:
                try:
                    client_info['queue'].put(f"{sender_addr}: {message}")
                except Exception as e:
                    print(f"Error broadcasting to {addr}: {e}")

def handle_messages(client_sock, addr):
    addr_str = f"{addr[0]}:{addr[1]}"
    try:
        while running:
            try:
                client_sock.settimeout(1.0)
                buff = client_sock.recv(1024)
                if not buff:  
                    break
                    
                msg = buff.decode('utf-8')
                print(f"Received from {addr_str}: {msg}")
                broadcast_message(addr_str, msg)
                
            except socket.timeout:
                continue
            except Exception as e:
                print(f"Error receiving from {addr_str}: {e}")
                break
    finally:
        cleanup_client(addr_str)
        client_sock.close()

def send_messages(client_sock, addr):
    addr_str = f"{addr[0]}:{addr[1]}"
    try:
        while running:
            try:
                message = clients[addr_str]['queue'].get(timeout=1.0)
                try:
                    client_sock.send(message.encode('utf-8'))
                except Exception as e:
                    print(f"Error sending to {addr_str}: {e}")
                    break
            except Empty:
                continue  
    except Exception as e:
        print(f"Error in send_messages for {addr_str}: {e}")
    finally:
        cleanup_client(addr_str)

def cleanup_client(addr_str):
    with messages_lock:
        if addr_str in clients:
            print(f"Client {addr_str} disconnected")
            del clients[addr_str]

def accept_clients(tcp_sock):
    global running
    while running:
        try:
            client_sock, addr = tcp_sock.accept()
            addr_str = f"{addr[0]}:{addr[1]}"
            print(f"[New connection]: from {addr_str}")
            
            with messages_lock:
                clients[addr_str] = {
                    'socket': client_sock,
                    'queue': Queue()
                }
            
            threading.Thread(target=handle_messages, args=(client_sock, addr)).start()
            threading.Thread(target=send_messages, args=(client_sock, addr)).start()
            
        except Exception as e:
            if running:
                print(f"Error accepting connection: {e}")

if __name__ == "__main__":
    try:
        tcp_sock = setup_tcp()
        accept_thread = threading.Thread(target=accept_clients, args=(tcp_sock,))
        accept_thread.start()
        
        try:
            while running:
                time.sleep(1)
        except KeyboardInterrupt:
            print("\nShutting down server...")
            running = False
            
        tcp_sock.close()
        accept_thread.join()
        
    except Exception as e:
        print(f"Server error: {e}")
        running = False