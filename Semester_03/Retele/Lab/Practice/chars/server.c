// client sends to the server a string and a character, the server returns
// a list of all the indexes of the character in the string

#include <arpa/inet.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#define PORT 6969
#define BUFFER_SIZE 1024

int main() {
  // Create socket
  int server_socket = socket(AF_INET, SOCK_STREAM, 0);
  if (server_socket == -1) {
    perror("Error creating socket\n");
    exit(1);
  }

  // Set socket options to reuse address (prevents "Address already in use"
  // error)
  int opt = 1;
  if (setsockopt(server_socket, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt))) {
    perror("Error setting socket options\n");
    exit(1);
  }

  // Initialize the server address structure
  struct sockaddr_in server_addr;
  memset(&server_addr, 0, sizeof(server_addr));
  server_addr.sin_family = AF_INET;
  server_addr.sin_port = htons(PORT);  // convert port to network byte order
  server_addr.sin_addr.s_addr = INADDR_ANY;  // accept connections from any ip

  // Bind socket to address
  if (bind(server_socket, (struct sockaddr*)&server_addr,
           sizeof(server_addr)) == -1) {
    perror("Error binding socket to address\n");
    close(server_socket);
    exit(1);
  }

  // Listen for incoming connections
  if (listen(server_socket, 5) == -1) {
    perror("Error listening for incoming connections\n");
    close(server_socket);
    exit(1);
  }

  for (;;) {
    struct sockaddr_in client_addr;
    socklen_t client_len = sizeof(client_addr);

    // Accept incoming connections
    int client_socket =
        accept(server_socket, (struct sockaddr*)&client_addr, &client_len);

    if (client_socket == -1) {
      perror("Error accepting incoming connection\n");
      continue;  // continue listening for incoming connections
    }

    printf("Accepted connection from %s:%d\n", inet_ntoa(client_addr.sin_addr),
           ntohs(client_addr.sin_port));

    // Handle client communication
    char buffer[BUFFER_SIZE];
    char c;

    // Receive string from client
    int bytes_received = recv(client_socket, buffer, BUFFER_SIZE - 1, 0);
    if (bytes_received <= 0) {
      perror("Error receiving string from client\n");
      close(client_socket);
      continue;  // continue listening for incoming connections
    }
    buffer[bytes_received] = '\0';  // null-terminate the string

    // Receive character from client
    int char_bytes_received = recv(client_socket, &c, 1, 0);
    if (char_bytes_received != 1) {
      perror("Error receiving character from client\n");
      close(client_socket);
      continue;  // continue listening for incoming connections
    }

    int indexes[BUFFER_SIZE];
    int idx = 0;

    for (int i = 0; i < bytes_received; i++) {
      if (buffer[i] == c) {
        indexes[idx++] = i;
      }
    }

    // Send indexes to client
    if (send(client_socket, &idx, sizeof(int), 0) == -1) {
      perror("Error sending number of indexes to client\n");
      close(client_socket);
      continue;  // continue listening for incoming connections
    }
    if (send(client_socket, indexes, idx * sizeof(int), 0) == -1) {
      perror("Error sending indexes to client\n");
      close(client_socket);
      continue;  // continue listening for incoming connections
    }

    close(client_socket);
    printf("Closed connection from %s:%d\n", inet_ntoa(client_addr.sin_addr),
           ntohs(client_addr.sin_port));
  }

  close(server_socket);
  return 0;
}