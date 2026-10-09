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

#define host "172.20.10.7"
#define port_udp 1234
#define port_tcp 1234

int running = 1;

int main() {
  srand(time(NULL));

  int udp_sock = socket(AF_INET, SOCK_DGRAM, 0);
  if (-1 == udp_sock) {
    perror("UDP SOCK error");
    exit(1);
  }

  struct sockaddr_in udp_addr;
  memset(&udp_addr, 0, sizeof(udp_addr));
  udp_addr.sin_family = AF_INET;
  udp_addr.sin_port = htons(port_udp);
  udp_addr.sin_addr.s_addr = inet_addr(host);

  int tcp_sock = socket(AF_INET, SOCK_STREAM, 0);

  struct sockaddr_in tcp_addr;
  memset(&tcp_addr, 0, sizeof(tcp_addr));
  tcp_addr.sin_addr.s_addr = inet_addr(host);
  tcp_addr.sin_port = htons(port_tcp);
  tcp_addr.sin_family = AF_INET;

  if (-1 == connect(tcp_sock, (struct sockaddr*)&tcp_addr, sizeof(tcp_addr))) {
    perror("error in tcp connect");
    close(udp_sock);
    exit(1);
  }

  while (running) {
    float pi_approx;
    int bytes_received = recv(tcp_sock, &pi_approx, sizeof(float), 0);
    if (bytes_received <= 0) {
      printf("Server disconnected. Shutting down...\n");
      running = 0;
      break;
    }
    printf("[SERVER]: I have PI approximation at %.6f\n", pi_approx);

    int numb1 = rand() % 100;
    int numb2 = rand() % 100;

    char buff[7];
    snprintf(buff, sizeof(buff), "%d;%d", numb1, numb2);

    if (-1 == sendto(udp_sock, buff, sizeof(buff), 0,
                     (struct sockaddr*)&udp_addr, sizeof(udp_addr))) {
      perror("error on send to udp");
      running = 0;
      break;
    }
    printf("Sent %d and %d\n", numb1, numb2);

    usleep(9000);
  }

  printf("Client shutting down...\n");
  close(udp_sock);
  close(tcp_sock);

  return 0;
}