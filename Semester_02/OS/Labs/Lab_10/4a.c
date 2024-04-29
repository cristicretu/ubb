#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

char *fifo1 = "./fifo1";
char *fifo2 = "./fifo2";

// a generates a random number, b tries to guess it, sends back and a responds
// with -1 or 1 or 0 if the guess is smaller, bigger or correct
int main() {
  if (0 > mkfifo(fifo1, 0666)) {
    perror("mkfifo");
    exit(1);
  }

  if (0 > mkfifo(fifo2, 0666)) {
    perror("mkfifo");
    exit(1);
  }

  int fd_write = open(fifo1, O_WRONLY);
  if (-1 == fd_write) {
    perror("open");
    exit(1);
  }

  int fd_read = open(fifo2, O_RDONLY);
  if (-1 == fd_read) {
    perror("open");
    exit(1);
  }

  srand(getpid());
  int num = rand() % 1001;
  printf("num: %d\n", num);

  while (1) {
    int b_num;
    if (-1 == read(fd_read, &b_num, sizeof(num))) {
      perror("read");
      exit(1);
    }

    if (b_num == num) {
      int response = 0;
      if (-1 == write(fd_write, &response, sizeof(response))) {
        perror("write");
        exit(1);
      }
      break;
    } else if (b_num < num) {
      int response = 1;
      if (-1 == write(fd_write, &response, sizeof(response))) {
        perror("write");
        exit(1);
      }
    } else {
      int response = -1;
      if (-1 == write(fd_write, &response, sizeof(response))) {
        perror("write");
        exit(1);
      }
    }
  }

  close(fd_write);
  close(fd_read);

  if (0 > unlink(fifo1)) {
    perror("unlink");
    exit(1);
  }

  if (0 > unlink(fifo2)) {
    perror("unlink");
    exit(1);
  }

  return 0;
}
