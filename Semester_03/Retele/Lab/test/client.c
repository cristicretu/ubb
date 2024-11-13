#include <arpa/inet.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#define BUFFER_SIZE 1024
#define IP "172.20.10.14"
#define TCP_PORT 1234

int main() {
  int tcp_sock = socket(AF_INET, SOCK_STREAM, 0);
  if (tcp_sock < 0) {
    perror("socket error");
    exit(1);
  }

  struct sockaddr_in tcp_addr;
  memset(&tcp_addr, 0, sizeof(tcp_addr));
  tcp_addr.sin_family = AF_INET;
  tcp_addr.sin_port = htons(TCP_PORT);
  tcp_addr.sin_addr.s_addr = inet_addr(IP);

  if (connect(tcp_sock, (struct sockaddr *)&tcp_addr, sizeof(tcp_addr)) < 0) {
    perror("connect error");
    exit(1);
  }

  char send_buffer[BUFFER_SIZE] = "cristi", recv_buffer[BUFFER_SIZE],
       server_result[BUFFER_SIZE], garbage[BUFFER_SIZE];

  int trolls = 0;

  while (1 && trolls < 3) {
    int num1, num2, op;
    float result = 0;
    char operator;
    num1 = rand() % 10 + 1;
    num2 = rand() % 10 + 1;
    op = rand() % 4;

    if (op == 0) {
      operator= '+';
      result = (float)num1 + (float)num2;
    }
    if (op == 1) {
      operator= '-';
      result = (float)num1 - (float)num2;
    }
    if (op == 2) {
      operator= '*';
      result = (float)num1 * (float)num2;
    }
    if (op == 3) {
      operator= '/';
      result = (float)num1 / (float)num2;
    }

    // printf("THIS IS THE CORRECT RESULT: %f\n", result);

    int messi = snprintf(send_buffer, sizeof(send_buffer), "%d%c%d",
                         num1, operator, num2);

    // printf("%s", send_buffer);

    if (send(tcp_sock, send_buffer, messi, 0) < 0) {
      perror("send error");
      exit(1);
    }

    int recv_len = recv(tcp_sock, recv_buffer, sizeof(recv_buffer) - 1, 0);
    if (recv_len < 0) {
      perror("recv error");
      exit(1);
    }
    recv_buffer[recv_len] = '\0';
    // sscanf(recv_buffer, "%s=%s", garbage, server_result);

    char *token = strtok(recv_buffer, "=");
    token = strtok(NULL, "=");

    printf("I got: %s\n", token);

    char result_str[6];
    //        snprintf(result_str, 4, "%f", result);
    gcvt(result, 6, result_str);

    printf("STRING RESULT IS: %s\n", result_str);

    if (strcmp(result_str, token) == 0) {
      printf("result is correct\n");
    } else {
      trolls++;
      printf("i got trolled\n");
    }
  }
}