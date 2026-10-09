#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

char *fifo1 = "./fifo1";
char *fifo2 = "./fifo2";

int main() {
  int fd_read = open(fifo1, O_RDONLY);
  if (-1 == fd_read) {
    perror("open");
    exit(1);
  }

  int fd_write = open(fifo2, O_WRONLY);
  if (-1 == fd_write) {
    perror("open");
    exit(1);
  }

  srand(getpid());

  int max_num = 1000;
  int min_num = 0;

  while (1) {
    int num = rand() % (max_num + 1);
    printf("B guess is: %d\n", num);

    if (-1 == write(fd_write, &num, sizeof(num))) {
      perror("write");
      exit(1);
    }

    int response;
    if (-1 == read(fd_read, &response, sizeof(response))) {
      perror("read");
      exit(1);
    }

    if (response == 0) {
      break;
    } else if (response == 1) {
      max_num = num - 1;
    } else {
      max_num = num + 1;
    }

    printf("A response is: %d\n", response);
  }

  close(fd_write);
  close(fd_read);

  return 0;
}
