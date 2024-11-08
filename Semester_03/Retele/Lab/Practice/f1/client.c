#include <arpa/inet.h>
#include <ctype.h>
#include <errno.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>

#define tcp_port 1234
#define udp_port 7777
#define host "127.0.0.1"

void clear_screen();
void print_table_header();
void print_table_row(int id, float speed, int distance, int laps);
void print_table_separator();
void display_leaderboard(int tcp_sock);

int main() {
  int udp_sock = socket(AF_INET, SOCK_DGRAM, 0);
  if (-1 == udp_sock) {
    perror("udp sock");
    exit(1);
  }

  int reuse = 1;
  if (-1 ==
      setsockopt(udp_sock, SOL_SOCKET, SO_REUSEPORT, &reuse, sizeof(reuse))) {
    perror("reuse");
    close(udp_sock);
    exit(1);
  }

  int tcp_sock = socket(AF_INET, SOCK_STREAM, 0);
  if (-1 == tcp_sock) {
    perror("tcp sock");
    close(udp_sock);
    exit(1);
  }

  if (-1 ==
      setsockopt(tcp_sock, SOL_SOCKET, SO_REUSEPORT, &reuse, sizeof(reuse))) {
    perror("reuse");
    close(udp_sock);
    close(tcp_sock);
    exit(1);
  }

  struct sockaddr_in udp_addr, tcp_addr;
  memset(&udp_addr, 0, sizeof(udp_addr));
  memset(&tcp_addr, 0, sizeof(tcp_addr));

  udp_addr.sin_family = AF_INET;
  udp_addr.sin_port = htons(udp_port);
  udp_addr.sin_addr.s_addr = INADDR_ANY;

  tcp_addr.sin_family = AF_INET;
  tcp_addr.sin_port = htons(tcp_port);
  tcp_addr.sin_addr.s_addr = INADDR_ANY;

  int driverId = 0;

  // Create a unique identifier for the shared memory segment
  key_t key = ftok(
      "/Users/huge/fun/ubb/Semester_03/Retele/Lab/Practice/f1/client.c", 'R');
  if (key == -1) {
    perror("ftok");
    close(udp_sock);
    close(tcp_sock);
    exit(1);
  }

  // Store shmid in wider scope so we can clean it up later
  int shmid;

  // Get the shared memory segment
  shmid = shmget(key, sizeof(int), 0666 | IPC_CREAT);
  if (shmid == -1) {
    perror("shmget");
    close(udp_sock);
    close(tcp_sock);
    exit(1);
  }

  // Attach to the shared memory segment
  int *shm = (int *)shmat(shmid, NULL, 0);
  if (shm == (int *)-1) {
    perror("shmat");
    close(udp_sock);
    close(tcp_sock);
    exit(1);
  }

  // Increment the driverId
  driverId = *shm;
  (*shm)++;

  // Detach from the shared memory segment
  if (shmdt(shm) == -1) {
    perror("shmdt");
    close(udp_sock);
    close(tcp_sock);
    exit(1);
  }

  int checkpoint = 0;
  char buff[8];

  printf("Going into main loop\n");

  while (1) {
    int time = rand() % 5;
    sleep(time);

    int size = sizeof(udp_addr);
    int buf_size = snprintf(buff, sizeof(buff), "%d;%d", driverId, checkpoint);

    printf("Sending %s\n", buff);
    if (-1 == sendto(udp_sock, buff, buf_size, 0, (struct sockaddr *)&udp_addr,
                     size)) {
      perror("send to udp");
      // Add cleanup before exit
      if (shmctl(shmid, IPC_RMID, NULL) == -1) {
        perror("shmctl");
      }
      close(udp_sock);
      close(tcp_sock);
      exit(1);
    }
    checkpoint = (checkpoint + 1) % 3;

    display_leaderboard(tcp_sock);
  }

  // Add cleanup for normal exit path
  if (shmctl(shmid, IPC_RMID, NULL) == -1) {
    perror("shmctl");
  }
  close(tcp_sock);
  close(udp_sock);
  return 0;
}

void display_leaderboard(int tcp_sock) {
  static int connected = 0;  // Track connection state
  struct sockaddr_in tcp_addr;

  // Only try to connect if not already connected
  if (!connected) {
    memset(&tcp_addr, 0, sizeof(tcp_addr));
    tcp_addr.sin_family = AF_INET;
    tcp_addr.sin_port = htons(tcp_port);
    inet_pton(AF_INET, host, &tcp_addr.sin_addr);

    if (connect(tcp_sock, (struct sockaddr *)&tcp_addr, sizeof(tcp_addr)) < 0) {
      if (errno != EISCONN) {  // Ignore "already connected" error
        perror("connect");
        return;
      }
    }
    connected = 1;
  }

  char buffer[256];
  clear_screen();
  print_table_header();

  // Send request for leaderboard
  if (send(tcp_sock, "GET", 3, 0) < 0) {
    if (errno == EPIPE) {
      // Handle broken pipe by reconnecting next time
      connected = 0;
      return;
    }
    perror("send");
    return;
  }

  while (1) {
    ssize_t n = recv(tcp_sock, buffer, sizeof(buffer) - 1, 0);
    if (n <= 0) break;

    buffer[n] = '\0';

    // Check for end marker
    if (strncmp(buffer, "END", 3) == 0) {
      break;
    }

    // Parse driver data
    int id, distance, laps;
    float speed;
    if (sscanf(buffer, "%d,%f,%d,%d", &id, &speed, &distance, &laps) == 4) {
      print_table_row(id, speed, distance, laps);
    }
  }
}

void clear_screen() {
  printf("\033[2J\033[H");  // ANSI escape codes to clear screen and move cursor
                            // to top
}

void print_table_header() {
  printf("┌────────┬────────────┬──────────┬──────┐\n");
  printf("│ Driver │ Avg Speed  │ Distance │ Laps │\n");
  printf("├────────┼────────────┼──────────┼──────┤\n");
}

void print_table_row(int id, float speed, int distance, int laps) {
  printf("│ %6d │ %8.2f   │ %8d │ %4d │\n", id, speed, distance, laps);
  print_table_separator();
}

void print_table_separator() {
  printf("└────────┴────────────┴──────────┴──────┘\n");
}