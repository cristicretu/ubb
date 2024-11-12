#include <arpa/inet.h>
#include <math.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>

#define TCP_PORT 1234
#define BUFFER_SIZE 64
#define IP "192.168.1.140"

int main() {
  int udp_sock = socket(AF_INET, SOCK_DGRAM, 0);
  if (udp_sock < 0) {
    perror("tcp sock erorr");
    exit(1);
  }

  struct sockaddr_in udp_addr;
  memset(&udp_addr, 0, sizeof(udp_addr));
  udp_addr.sin_port = htons(TCP_PORT);
  udp_addr.sin_family = AF_INET;
  udp_addr.sin_addr.s_addr = INADDR_ANY;

  int param = 1;
  if (setsockopt(udp_sock, SOL_SOCKET, SO_REUSEADDR, &param, sizeof(param)) <
          0 ||
      setsockopt(udp_sock, SOL_SOCKET, SO_REUSEPORT, &param, sizeof(param)) <
          0 ||
      setsockopt(udp_sock, SOL_SOCKET, SO_BROADCAST, &param, sizeof(param)) <
          0) {
    perror("setsockopt failed");
    close(udp_sock);
    exit(1);
  }

  if (bind(udp_sock, (struct sockaddr*)&udp_addr, sizeof(udp_addr)) < 0) {
    perror("error on bind");
    exit(1);
  }

  int tcp_sock = socket(AF_INET, SOCK_STREAM, 0);
  if (tcp_sock < 0) {
    perror("tcp sock erorr");
    exit(1);
  }

  struct sockaddr_in tcp_addr;
  memset(&tcp_addr, 0, sizeof(udp_addr));
  tcp_addr.sin_port = htons(TCP_PORT);
  tcp_addr.sin_family = AF_INET;
  tcp_addr.sin_addr.s_addr = inet_addr(IP);

  if (connect(tcp_sock, (struct sockaddr*)&tcp_addr, sizeof(tcp_addr)) < 0) {
    perror("connect erorr");
    exit(1);
  }

  char buffer[BUFFER_SIZE];
  socklen_t addr_len = sizeof(udp_addr);

  int recv_len = recvfrom(udp_sock, buffer, BUFFER_SIZE, 0,
                          (struct sockaddr*)&udp_addr, &addr_len);
  buffer[recv_len] = '\0';

  printf("buffer: %s\n", buffer);
}