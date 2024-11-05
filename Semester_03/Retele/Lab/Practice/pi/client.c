#include <math.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>

#define host "localhost"
#define port_udp 1234
#define port_tcp 7777

int running = 1;

int main() {
  srand48(time(NULL));

  int udp_sock = socket(AF_INET, SOCK_DGRAM, 0);
  if (-1 == udp_sock) {
    perror("UDP SOCK error");
    exit(1);
  }

  struct sockaddr_in udp_addr;
  memset(&udp_addr, 0, sizeof(udp_addr));
  udp_addr.sin_family = AF_INET;
  udp_addr.sin_port = htons(port_udp);
  udp_addr.sin_addr.s_addr = INADDR_ANY;

  while (running) {
    int numb1 = rand() % 100;
    int numb2 = rand() % 100;

    char buff[7];
    snprintf(buff, sizeof(buff), "%d;%d", numb1, numb2);

    if (-1 == sendto(udp_sock, buff, sizeof(buff), 0,
                     (struct sockaddr*)&udp_addr, sizeof(udp_addr))) {
      perror("error on send to udp");
      close(udp_sock);
      exit(1);
    }
    printf("Sent %d and %d\n", numb1, numb2);

    srand48(time(NULL));
    sleep(1);
  }

  close(udp_sock);

  return 0;
}