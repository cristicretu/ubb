#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>

int main(int argc, char *argv[]) {
  int p2c = open("./fifo1", O_RDONLY);
  if (-1 == p2c) {
    perror("open");
    exit(1);
  }

  int c2p = open("./fifo2", O_WRONLY);
  if (-1 == c2p) {
    perror("open");
    close(p2c);
    exit(1);
  }

  int size;

  if (-1 == read(p2c, &size, sizeof(int))) {
    perror("read");
    close(p2c);
    close(c2p);
    exit(1);
  }

  int sum = 0;

  for (int i = 0; i < size; i++) {
    int x;
    if (-1 == read(p2c, &x, sizeof(int))) {
      perror("read");
      close(p2c);
      close(c2p);
      exit(1);
    }
    sum += x;
  }

  if (-1 == write(c2p, &sum, sizeof(int))) {
    perror("write");
    close(p2c);
    close(c2p);
    exit(1);
  }

  close(p2c);
  close(c2p);
  return 0;
}
