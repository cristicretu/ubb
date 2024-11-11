#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>

#define UDP_PORT 7777
#define TCP_PORT 1234
#define IP "172.20.10.14"
#define N 10
#define ARRAY_SIZE 1024

void print_array(const char* buffer) {
  char* token;
  char* rest = strdup(buffer);
  int row = 0;

  // Print the array in a grid format
  printf("\nReceived Array:\n");
  while ((token = strsep(&rest, "|")) != NULL && row < N) {
    char* col_token;
    char* col_rest = token;
    while ((col_token = strsep(&col_rest, ",")) != NULL) {
      printf("%s ", col_token);
    }
    printf("\n");
    row++;
  }
  printf("\n");
  free(rest);
}

int main() {
  // Create UDP socket
  int udp_socket = socket(AF_INET, SOCK_DGRAM, 0);
  if (udp_socket < 0) {
    perror("error on udp_socket");
    exit(1);
  }

  // Enable broadcast and reuse address
  int broadcast = 1;
  int reuse = 1;
  if (setsockopt(udp_socket, SOL_SOCKET, SO_BROADCAST, &broadcast,
                 sizeof(broadcast)) < 0) {
    perror("setsockopt (SO_BROADCAST)");
    exit(1);
  }
  if (setsockopt(udp_socket, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) <
      0) {
    perror("setsockopt (SO_REUSEADDR)");
    exit(1);
  }

  // Setup UDP address for binding
  struct sockaddr_in udp_addr;
  memset(&udp_addr, 0, sizeof(udp_addr));
  udp_addr.sin_family = AF_INET;
  udp_addr.sin_port = htons(UDP_PORT);
  udp_addr.sin_addr.s_addr = INADDR_ANY;  // Listen on all interfaces

  // Bind the socket
  if (bind(udp_socket, (struct sockaddr*)&udp_addr, sizeof(udp_addr)) < 0) {
    perror("bind failed");
    exit(1);
  }

  printf("UDP Receiver listening on port %d...\n", UDP_PORT);

  char receive_buffer[ARRAY_SIZE];
  struct sockaddr_in sender_addr;
  socklen_t addr_len = sizeof(sender_addr);

  while (1) {
    printf("Waiting for message...\n");

    // Clear the receive buffer
    memset(receive_buffer, 0, ARRAY_SIZE);

    int udp_bytes = recvfrom(udp_socket, receive_buffer, ARRAY_SIZE - 1, 0,
                             (struct sockaddr*)&sender_addr, &addr_len);

    if (udp_bytes < 0) {
      perror("udp receive error");
      continue;
    }

    receive_buffer[udp_bytes] = '\0';

    printf("Received %d bytes from %s:%d\n", udp_bytes,
           inet_ntoa(sender_addr.sin_addr), ntohs(sender_addr.sin_port));

    // Print the received array in a readable format
    print_array(receive_buffer);
  }

  close(udp_socket);
  return 0;
}