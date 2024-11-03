#include <arpa/inet.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#define PORT 7777
#define TCP_PORT 1234
#define BUFFER_SIZE 1024

int evaluate_expression(const char* expr) {
  int a, b;
  char op;
  if (sscanf(expr, "%d%c%d", &a, &op, &b) != 3) {
    return 0;
  }

  switch (op) {
    case '+':
      return a + b;
    case '-':
      return a - b;
    case '*':
      return a * b;
    case '/':
      return b != 0 ? a / b : 0;
    default:
      return 0;
  }
}

int main() {
  srand(time(NULL));

  // Create UDP socket for receiving broadcast
  int udp_sock = socket(AF_INET, SOCK_DGRAM, 0);
  if (udp_sock < 0) {
    perror("UDP socket creation failed");
    exit(1);
  }

  // Enable broadcast and address reuse
  int broadcast_enable = 1;
  int reuse = 1;
  if (setsockopt(udp_sock, SOL_SOCKET, SO_BROADCAST, &broadcast_enable,
                 sizeof(broadcast_enable)) < 0) {
    perror("Error setting broadcast option");
    close(udp_sock);
    exit(1);
  }
  if (setsockopt(udp_sock, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) <
      0) {
    perror("Error setting reuse option");
    close(udp_sock);
    exit(1);
  }

  // Setup UDP address for receiving broadcasts
  struct sockaddr_in udp_addr;
  memset(&udp_addr, 0, sizeof(udp_addr));
  udp_addr.sin_family = AF_INET;
  udp_addr.sin_port = htons(PORT);
  udp_addr.sin_addr.s_addr = INADDR_ANY;

  // Bind is necessary for receiving broadcasts
  if (bind(udp_sock, (struct sockaddr*)&udp_addr, sizeof(udp_addr)) < 0) {
    perror("UDP bind failed");
    close(udp_sock);
    exit(1);
  }

  printf("Waiting for questions...\n");

  // Receive broadcast message
  char buffer[BUFFER_SIZE];
  struct sockaddr_in server_addr;
  socklen_t server_len = sizeof(server_addr);

  ssize_t recv_len = recvfrom(udp_sock, buffer, BUFFER_SIZE - 1, 0,
                              (struct sockaddr*)&server_addr, &server_len);
  if (recv_len < 0) {
    perror("UDP receive failed");
    close(udp_sock);
    exit(1);
  }

  buffer[recv_len] = '\0';
  printf("Received questions: %s\n", buffer);
  close(udp_sock);

  // Parse expressions and calculate answers
  char* expressions[3];
  int answers[3];
  int num_questions = 0;

  char* token = strtok(buffer, ";");
  while (token != NULL && num_questions < 3) {
    expressions[num_questions] = strdup(token);
    answers[num_questions] = evaluate_expression(token);
    printf("Question %d: %s = %d\n", num_questions, token,
           answers[num_questions]);
    num_questions++;
    token = strtok(NULL, ";");
  }

  // Create TCP socket for submitting answers
  int tcp_sock = socket(AF_INET, SOCK_STREAM, 0);
  if (tcp_sock < 0) {
    perror("TCP socket creation failed");
    exit(1);
  }

  // Setup TCP address
  struct sockaddr_in tcp_addr;
  memset(&tcp_addr, 0, sizeof(tcp_addr));
  tcp_addr.sin_family = AF_INET;
  tcp_addr.sin_port = htons(TCP_PORT);
  tcp_addr.sin_addr.s_addr = inet_addr("127.0.0.1");  // Connect to localhost

  // Connect to server
  if (connect(tcp_sock, (struct sockaddr*)&tcp_addr, sizeof(tcp_addr)) < 0) {
    perror("TCP connection failed");
    close(tcp_sock);
    exit(1);
  }

  // Send answers
  char answer_buffer[32];
  for (int i = 0; i < num_questions; i++) {
    snprintf(answer_buffer, sizeof(answer_buffer), "%d.%d", i, answers[i]);
    if (send(tcp_sock, answer_buffer, strlen(answer_buffer), 0) < 0) {
      perror("Failed to send answer");
      close(tcp_sock);
      exit(1);
    }
    // Small delay between sends to prevent message coalescence
    usleep(100000);
  }

  // Receive score
  memset(buffer, 0, BUFFER_SIZE);
  recv_len = recv(tcp_sock, buffer, BUFFER_SIZE - 1, 0);
  if (recv_len > 0) {
    buffer[recv_len] = '\0';
    printf("Score received: %s\n", buffer);
  }

  // Cleanup
  close(tcp_sock);
  for (int i = 0; i < num_questions; i++) {
    free(expressions[i]);
  }

  return 0;
}