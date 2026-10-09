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

#define TCP_PORT 1234
char buffer[1024];

int main() {
  int server_socket, client_socket;

  server_socket = socket(AF_INET, SOCK_STREAM, 0);
  if (server_socket < 0) {
    perror("tcp_socket error");
    exit(1);
  }

  struct sockaddr_in server_addr;
  memset(&server_addr, 0, sizeof(server_addr));
  server_addr.sin_family = AF_INET;
  server_addr.sin_port = htons(TCP_PORT);
  server_addr.sin_addr.s_addr = inet_addr("192.168.1.138");

  bind(server_socket, (struct sockaddr*)&server_addr, sizeof(server_addr));
  listen(server_socket, 5);

  client_socket = accept(server_socket, NULL, NULL);

  send(client_socket, "this is server", 1024, 0);
  recv(client_socket, buffer, 1024, 0);
  printf("Received: %s\n", buffer);

  close(client_socket);
  close(server_socket);
}