#include <arpa/inet.h>
#include <netinet/in.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#define HOST "127.0.0.1"
#define UDP_PORT 7777
#define TCP_PORT 1234
#define BUFFER_SIZE 1024

int running = 1;
int udp_sock, tcp_sock;

void* udp_listener(void* arg) {
  char buffer[BUFFER_SIZE];
  while (running) {
    memset(buffer, 0, BUFFER_SIZE);
    ssize_t received =
        recvfrom(udp_sock, buffer, BUFFER_SIZE - 1, 0, NULL, NULL);
    if (received > 0) {
      printf("\rBroadcast: %s\nEnter message: ", buffer);
      fflush(stdout);
    }
  }
  return NULL;
}

int main() {
  udp_sock = socket(AF_INET, SOCK_DGRAM, 0);
  if (udp_sock == -1) {
    perror("UDP socket creation failed");
    exit(1);
  }

  int reuse = 1;
  if (setsockopt(udp_sock, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) ==
      -1) {
    perror("UDP setsockopt SO_REUSEADDR failed");
    exit(1);
  }
  if (setsockopt(udp_sock, SOL_SOCKET, SO_REUSEPORT, &reuse, sizeof(reuse)) ==
      -1) {
    perror("UDP setsockopt SO_REUSEPORT failed");
    exit(1);
  }

  tcp_sock = socket(AF_INET, SOCK_STREAM, 0);
  if (tcp_sock == -1) {
    perror("TCP socket creation failed");
    exit(1);
  }

  struct sockaddr_in udp_addr;
  memset(&udp_addr, 0, sizeof(udp_addr));
  udp_addr.sin_family = AF_INET;
  udp_addr.sin_port = htons(UDP_PORT);
  udp_addr.sin_addr.s_addr = INADDR_ANY;

  struct sockaddr_in tcp_addr;
  memset(&tcp_addr, 0, sizeof(tcp_addr));
  tcp_addr.sin_family = AF_INET;
  tcp_addr.sin_port = htons(TCP_PORT);
  tcp_addr.sin_addr.s_addr = inet_addr(HOST);

  if (connect(tcp_sock, (struct sockaddr*)&tcp_addr, sizeof(tcp_addr)) == -1) {
    perror("TCP connect failed");
    exit(1);
  }

  if (bind(udp_sock, (struct sockaddr*)&udp_addr, sizeof(udp_addr)) == -1) {
    perror("UDP bind failed");
    exit(1);
  }

  pthread_t udp_thread;
  if (pthread_create(&udp_thread, NULL, udp_listener, NULL) != 0) {
    perror("Thread creation failed");
    exit(1);
  }

  char buffer[BUFFER_SIZE];
  printf("Connected to chat. Type 'exit' to quit.\n");

  while (running) {
    printf("Enter message: ");
    if (fgets(buffer, BUFFER_SIZE, stdin) == NULL) {
      break;
    }

    if (strncmp(buffer, "exit", 4) == 0) {
      running = 0;
      break;
    }

    if (send(tcp_sock, buffer, strlen(buffer), 0) == -1) {
      perror("TCP send failed");
      break;
    }
  }

  running = 0;
  pthread_join(udp_thread, NULL);
  close(udp_sock);
  close(tcp_sock);

  return 0;
}