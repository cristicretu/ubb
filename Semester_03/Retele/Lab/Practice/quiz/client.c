#include <arpa/inet.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>

#define PORT 7777
#define TCP_PORT 1234
#define BUFFER 1024

int evalueaza(char* str) {
  int first, second;
  char sep[] = {'+', '-', '*', '/'};
  char* tk = strtok(str, sep);

  first = atoi(tk);
  tk = strtok(NULL, sep);

  second = atoi(tk);

  int op_index = strcspn(str, sep);
  char op = str[op_index];

  switch (op) {
    case '+':
      return first + second;
    case '-':
      return first - second;
    case '*':
      return first * second;
    case '/':
      return first / second;
    default:
      return -1;
  }
}

int main() {
  srand(time(NULL));
  // create the udp socket
  int udp_sock = socket(AF_INET, SOCK_DGRAM, 0);
  if (-1 == udp_sock) {
    perror("error in udp broadcast");
    exit(1);
  }

  int brodcast_enable = 1;
  setsockopt(udp_sock, SOL_SOCKET, SO_BROADCAST, &brodcast_enable,
             sizeof(brodcast_enable));

  struct sockaddr_in udp_addr;
  memset(&udp_addr, 0, sizeof(udp_addr));
  udp_addr.sin_family = AF_INET;
  udp_addr.sin_port = htons(PORT);
  udp_addr.sin_addr.s_addr = INADDR_ANY;

  printf("Waiting for questions...\n");

  char buffer[BUFFER];
  size_t recv_len = recv(udp_sock, buffer, BUFFER - 1, 0);
  if (-1 == recv_len) {
    perror("UDP receive failed");
    close(udp_sock);
    exit(1);
  }

  buffer[BUFFER] = '\0';
  printf("Received questions: %s\n", buffer);

  char* token = strtok(buffer, ";");

  int answers[3];
  int idx = 0;
  while (token != NULL) {
    int loss = rand() % 2 - 1;
    answers[idx++] = evalueaza(token) + loss;
    token = strtok(NULL, ";");
  }

  close(udp_sock);

  int tcp_sock = socket(AF_INET, SOCK_STREAM, 0);
  if (-1 == tcp_sock) {
    perror("error in tcp socket");
    exit(1);
  }

  struct sockaddr_in tcp_addr;
  memset(&tcp_addr, 0, sizeof(tcp_addr));
  tcp_addr.sin_family = AF_INET;
  tcp_addr.sin_port = htons(TCP_PORT);
  tcp_addr.sin_addr.s_addr = inet_addr("192.168.1.102");

  if (-1 == connect(tcp_sock, (struct sockaddr*)&tcp_addr, sizeof(tcp_addr))) {
    perror("error in tcp connect");
    close(tcp_sock);
    exit(1);
  }

  char response[BUFFER];
  memset(response, 0, BUFFER);

  for (int i = 0; i < 3; i++) {
    sprintf(response, "%d.%d", i, answers[i]);
    if (-1 == send(tcp_sock, response, strlen(response), 0)) {
      perror("error in tcp send");
      close(tcp_sock);
      exit(1);
    }
    memset(response, 0, BUFFER);
  }

  memset(response, 0, BUFFER);
  if (-1 == recv(tcp_sock, response, BUFFER - 1, 0)) {
    perror("error in tcp receive");
    close(tcp_sock);
    exit(1);
  }

  printf("Received: %s\n", response);

  return 0;
}
