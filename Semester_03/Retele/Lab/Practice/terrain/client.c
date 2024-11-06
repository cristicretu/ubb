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
#define N 5
#define host "127.0.0.1"

int main() {
  int udp_sock = socket(AF_INET, SOCK_DGRAM, 0);
  if (-1 == udp_sock) {
    perror("Error in udp sock");
    exit(1);
  }

  int tcp_sock = socket(AF_INET, SOCK_STREAM, 0);
  if (-1 == tcp_sock) {
    perror("error in tcp sock");
    close(udp_sock);
    exit(1);
  }

  struct sockaddr_in udp_addr, tcp_addr;
  memset(&udp_addr, 0, sizeof(udp_addr));
  memset(&tcp_addr, 0, sizeof(tcp_addr));

  udp_addr.sin_port = htons(port_udp);
  udp_addr.sin_family = AF_INET;
  udp_addr.sin_addr.s_addr = INADDR_ANY;

  if (-1 == bind(udp_sock, (struct sockaddr*)&udp_addr, sizeof(udp_addr))) {
    perror("can't bind to udp");
    close(udp_sock);
    close(tcp_sock);
    exit(1);
  }

  tcp_addr.sin_port = htons(port_tcp);
  tcp_addr.sin_family = AF_INET;
  tcp_addr.sin_addr.s_addr = INADDR_ANY;

  char buff[N + 1];

  printf("got here\n");

  while (1) {
    // send random char and index after 5 sec

    printf("got inside the while\n");
    // listen for the new configuration
    socklen_t addr_len = sizeof(udp_addr);
    size_t recv_len;
    if (-1 == (recv_len = recvfrom(udp_sock, buff, sizeof(buff), 0,
                                   (struct sockaddr*)&udp_addr, &addr_len))) {
      perror("Error on receiving from UDP");
      close(udp_sock);
      close(tcp_sock);
      exit(1);
    }
    buff[recv_len] = '\0';

    printf("got past the recvfrom\n");
    printf("Received from brodcast: %s\n", buff);
    break;
  }

  close(udp_sock);
  close(tcp_sock);
  return 0;
}