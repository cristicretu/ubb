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
  if (-1 == mkfifo("./fifo1", 0600 | O_CREAT)) {
    perror("mkfifo");
    exit(1);
  }
  if (-1 == mkfifo("./fifo2", 0600 | O_CREAT)) {
    perror("mkfifo");
    exit(1);
  }

  int p2c = open("./fifo1", O_WRONLY);
  if (-1 == p2c) {
    perror("open");
    exit(1);
  }

  int c2p = open("./fifo2", O_RDONLY);
  if (-1 == c2p) {
    perror("open");
    close(p2c);
    exit(1);
  }

  srandom(time(NULL));
  int n;
  scanf("%d", &n);

  if (n > 0) {
    int *arr = (int *)malloc(n * sizeof(int));

    for (int i = 0; i < n; i++) {
      arr[i] = rand() % 100;
      printf("%d ", arr[i]);
    }
    printf("\n");

    if (-1 == write(p2c, &n, sizeof(int))) {
      perror("write");
      close(p2c);
      free(arr);
      wait(0);
      exit(1);
    }

    for (int i = 0; i < n; i++) {
      if (-1 == write(p2c, &arr[i], sizeof(int))) {
        perror("write");
        close(p2c);
        free(arr);
        wait(0);
        exit(1);
      }
    }
    free(arr);

    int sum = 0;
    if (-1 == read(c2p, &sum, sizeof(int))) {
      perror("read");
      close(p2c);
      close(c2p);
      wait(0);
      exit(1);
    }
    printf("Sum: %d\n", sum);
  }
  wait(NULL);
  close(p2c);
  close(c2p);

  unlink("./fifo1");
  unlink("./fifo2");
  return 0;
}
