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
#define SERVER_IP "127.0.0.1"

int main() {
  // Create socket
  int client_socket = socket(AF_INET, SOCK_STREAM, 0);
  // ipv4, tcp, default protocol (tcp)
  if (client_socket == -1) {
    perror("Error creating socket");
    exit(1);
  }

  // socket options are only needed for the server

  struct sockaddr_in server_addr;
  server_addr.sin_family = AF_INET;
  server_addr.sin_port = htons(PORT);
  server_addr.sin_addr.s_addr = inet_addr(SERVER_IP);
  if (-1 == connect(client_socket, (struct sockaddr *)&server_addr,
                    sizeof(server_addr))) {
    perror("Error connecting to server");
    close(client_socket);
    exit(1);
  }

  // get input from the user
  char buffer[BUFFER_SIZE];
  char c;

  printf("Enter a message: ");
  if (fgets(buffer, BUFFER_SIZE, stdin) == NULL) {
    perror("Error reading input");
    close(client_socket);
    exit(1);
  }
  buffer[strcspn(buffer, "\n")] = 0;

  printf("Enter a char:");
  c = getchar();
  getchar();  // consume the newline character

  if (-1 == send(client_socket, buffer, strlen(buffer), 0)) {
    perror("Error sending message");
    close(client_socket);
    exit(1);
  }

  if (-1 == send(client_socket, &c, 1, 0)) {
    perror("Error sending char");
    close(client_socket);
    exit(1);
  }

  // Receive from the server

  int index_count = 0;

  if (-1 == recv(client_socket, &index_count, sizeof(index_count), 0)) {
    perror("Error receiving index count");
    close(client_socket);
    exit(1);
  }

  if (index_count == 0) {
    printf("Character not found in the message\n");
  } else {
    int indexes[index_count];
    if (-1 == recv(client_socket, indexes, index_count * sizeof(int), 0)) {
      perror("Error receiving indexes");
      close(client_socket);
      exit(1);
    }

    printf("Character found at indexes: ");
    for (int i = 0; i < index_count; i++) {
      printf("%d ", indexes[i]);
    }
    printf("\n");
  }

  close(client_socket);
  return 0;
}