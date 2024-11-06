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
#define N 30
#define host "127.0.0.1"

int main() {
  srand(time(NULL));

  // Create UDP socket
  int udp_sock = socket(AF_INET, SOCK_DGRAM, 0);
  if (udp_sock == -1) {
    perror("Error creating UDP socket");
    exit(1);
  }

  // Create TCP socket
  int tcp_sock = socket(AF_INET, SOCK_STREAM, 0);
  if (tcp_sock == -1) {
    perror("Error creating TCP socket");
    close(udp_sock);
    exit(1);
  }

  struct sockaddr_in udp_addr, tcp_addr;
  memset(&udp_addr, 0, sizeof(udp_addr));
  memset(&tcp_addr, 0, sizeof(tcp_addr));

  // Configure UDP address for binding
  udp_addr.sin_family = AF_INET;
  udp_addr.sin_port = htons(port_udp);
  udp_addr.sin_addr.s_addr = INADDR_ANY;  // Bind to all interfaces

  // Set socket options to allow multiple bindings
  int reuse = 1;
  if (setsockopt(udp_sock, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) ==
      -1) {
    perror("setsockopt SO_REUSEADDR failed");
    close(udp_sock);
    close(tcp_sock);
    exit(1);
  }

#ifdef SO_REUSEPORT
  if (setsockopt(udp_sock, SOL_SOCKET, SO_REUSEPORT, &reuse, sizeof(reuse)) ==
      -1) {
    perror("setsockopt SO_REUSEPORT failed");
    close(udp_sock);
    close(tcp_sock);
    exit(1);
  }
#endif

  // Bind UDP socket
  if (bind(udp_sock, (struct sockaddr *)&udp_addr, sizeof(udp_addr)) == -1) {
    perror("Can't bind to UDP port");
    close(udp_sock);
    close(tcp_sock);
    exit(1);
  }

  // Configure TCP address for connecting
  tcp_addr.sin_family = AF_INET;
  tcp_addr.sin_port = htons(port_tcp);
  if (inet_pton(AF_INET, host, &tcp_addr.sin_addr) <= 0) {
    perror("Invalid TCP address");
    close(udp_sock);
    close(tcp_sock);
    exit(1);
  }

  // Connect TCP socket to server
  if (connect(tcp_sock, (struct sockaddr *)&tcp_addr, sizeof(tcp_addr)) == -1) {
    perror("Can't connect to TCP server");
    close(udp_sock);
    close(tcp_sock);
    exit(1);
  }

  char buff[N + 1];
  memset(buff, 'u', sizeof(buff));

  char *letters = "abcdefghijklmnopqrstuvwxyz";

  while (1) {
    int index = rand() % N;
    while (buff[index] != 'u') {
      index = rand() % N;
    }

    int chary = rand() % 26;
    char buf[3];
    unsigned short idx = (unsigned short)index;
    buf[0] = (idx >> 8) & 0xFF;
    buf[1] = idx & 0xFF;
    buf[2] = letters[chary];

    printf("Sending - Index: %d, Char: %c\n", index, letters[chary]);

    if (send(tcp_sock, buf, 3, 0) == -1) {
      perror("TCP send failed");
      break;
    }

    // Update local buffer
    buff[index] = letters[chary];

    // Listen for the new configuration via UDP
    socklen_t addr_len = sizeof(udp_addr);
    ssize_t recv_len = recvfrom(udp_sock, buff, sizeof(buff) - 1, 0,
                                (struct sockaddr *)&udp_addr, &addr_len);
    if (recv_len == -1) {
      perror("Error receiving from UDP");
      break;
    }
    buff[recv_len] = '\0';

    printf("Received from broadcast: %s\n", buff);

    int has_unexplored = 0;
    for (int i = 0; i < N; i++) {
      if (buff[i] == 'u') {
        has_unexplored = 1;
        break;
      }
    }

    if (!has_unexplored) {
      printf("Array fully explored, shutting down...\n");
      break;
    }

    usleep(1000);  // Sleep for 10ms
  }

  close(udp_sock);
  close(tcp_sock);
  return 0;
}
