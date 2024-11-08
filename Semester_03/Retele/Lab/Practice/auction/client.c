#include <arpa/inet.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>

#define port_udp 7777
#define port_tcp 1234
#define host "127.0.0.1"

int main() {
  srand(time(NULL));
  // Setup sockets for UDP and TCP
  int udp_sock = socket(AF_INET, SOCK_DGRAM, 0);
  if (udp_sock == -1) {
    perror("Creating UDP SOCK");
    exit(1);
  }

  int tcp_sock = socket(AF_INET, SOCK_STREAM, 0);
  if (tcp_sock == -1) {
    perror("Creating TCP sock");
    close(udp_sock);
    exit(1);
  }

  // Set socket options
  // int broadcast = 1;
  // if (-1 == setsockopt(udp_sock, SOL_SOCKET, SO_BROADCAST, &broadcast,
  //                      sizeof(broadcast))) {
  //   perror("setting brodcast opt");
  //   close(udp_sock);
  //   exit(1);
  // }

  // int reuse = 1;
  // if (-1 ==
  //     setsockopt(udp_sock, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)))
  //     {
  //   perror("setting reuse opt");
  //   close(udp_sock);
  //   exit(1);
  // }

  struct sockaddr_in udp_addr, tcp_addr;
  memset(&udp_addr, 0, sizeof(udp_addr));
  udp_addr.sin_family = AF_INET;
  udp_addr.sin_port = htons(port_udp);
  udp_addr.sin_addr.s_addr = INADDR_ANY;

  memset(&tcp_addr, 0, sizeof(tcp_addr));
  tcp_addr.sin_family = AF_INET;
  tcp_addr.sin_port = htons(port_tcp);
  tcp_addr.sin_addr.s_addr = INADDR_ANY;

  // Bind udp, then connect tcp
  if (-1 == bind(udp_sock, (struct sockaddr*)&udp_addr, sizeof(udp_addr))) {
    perror("Binding to udp");
    close(tcp_sock);
    close(udp_sock);
    exit(1);
  }

  if (-1 == connect(tcp_sock, (struct sockaddr*)&tcp_addr, sizeof(tcp_addr))) {
    perror("Connecting to tcp");
    close(tcp_sock);
    close(udp_sock);
    exit(1);
  }

  int current_price = 0;
  int my_max_price = rand() % 300 + 500;

  while (1) {
    char buff[4];
    socklen_t addr_len = sizeof(udp_addr);
    if (-1 == recvfrom(udp_sock, &buff, sizeof(buff), 0,
                       (struct sockaddr*)&udp_addr, &addr_len)) {
      perror("Error on receiving from UDP");
      close(tcp_sock);
      close(udp_sock);
      exit(1);
    }

    int curr_price = atoi(buff);
    printf("Current price is %d\n", curr_price);
  }

  close(tcp_sock);
  close(udp_sock);
  return 0;
}