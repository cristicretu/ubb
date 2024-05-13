#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

int abs(int a) { return a >= 0 ? a : -a; }

int main(int argc, char **argv) {
  int a2d[2], b2d[2], c2d[2];
  int d2b[2], d2c[2];

  if (-1 == pipe(a2d) || -1 == pipe(b2d) || -1 == pipe(c2d)) {
    perror("error on some pipes");
    exit(1);
  }

  if (-1 == pipe(d2c) || -1 == pipe(d2b)) {
    perror("error on other pipes");
    exit(1);
  }

  int a = rand() % 11 + 10;
  printf("A is sending %d\n", a);

  if (-1 == write(a2d[1], &a, sizeof(int))) {
    perror("writing a to d");
    exit(1);
  }

  pid_t b = fork();

  if (b < 0) {
    perror("b");
    exit(1);
  } else if (b == 0) {
    close(b2d[0]);
    close(d2b[1]);
    srand(getpid());

    while (1) {
      int num = rand() % 200 + 1;
      printf("B is sending %d\n", num);

      if (-1 == write(b2d[1], &num, sizeof(int))) {
        perror("b2d");
        close(d2b[0]), close(b2d[1]);
        exit(1);
      }

      int keepgoing;

      if (-1 == read(d2b[0], &keepgoing, sizeof(int))) {
        perror("d2b");
        close(d2b[0]), close(b2d[1]);
        exit(1);
      }

      if (keepgoing == 0) {
        break;
      }
    }

    close(d2b[0]);
    close(b2d[1]);
  }

  pid_t c = fork();

  if (c < 0) {
    perror("c");
    exit(1);
  } else if (c == 0) {
    close(c2d[0]);
    close(d2c[1]);
    while (1) {
      int num = rand() % 200 + 1;
      printf("C is sending %d\n", num);

      if (-1 == write(c2d[1], &num, sizeof(int))) {
        perror("c2d");
        close(c2d[1]), close(d2c[0]);
        exit(1);
      }

      int keepgoing;

      if (-1 == read(d2c[0], &keepgoing, sizeof(int))) {
        perror("d2c");
        close(c2d[1]), close(d2c[0]);
        exit(1);
      }

      if (keepgoing == 0) {
        break;
      }
    }
    close(c2d[1]);
    close(d2c[0]);
  }

  pid_t d = fork();

  if (d < 0) {
    perror("d");
    exit(1);
  } else if (d == 0) {
    // read from a first
    close(a2d[1]);
    close(b2d[1]);
    close(c2d[1]);
    int a_num;

    if (-1 == read(a2d[0], &a_num, sizeof(int))) {
      perror("a2d");
      close(a2d[0]), close(b2d[0]), close(c2d[0]);
      exit(1);
    }

    while (1) {
      // read b and c

      int b_num, c_num;

      if (-1 == read(b2d[0], &b_num, sizeof(int)) ||
          -1 == read(c2d[0], &c_num, sizeof(int))) {
        perror("on reading b or c from d");
        close(a2d[0]), close(b2d[0]), close(c2d[0]);
        exit(1);
      }

      int diff = abs(b_num - c_num);
      printf(
          "D received %d and %d, the absolute diff is %d, and the answer is "
          "%d\n",
          b_num, c_num, diff, diff <= a_num);

      int keepgoing = diff <= a_num;
      keepgoing = !keepgoing;

      // signal b and c to stop
      if (-1 == write(d2b[1], &keepgoing, sizeof(int)) ||
          -1 == write(d2c[1], &keepgoing, sizeof(int))) {
        perror("on writing to b or c");
        close(a2d[0]), close(b2d[0]), close(c2d[0]);
        exit(1);
      }

      if (diff <= a_num) {
        break;
      }
    }
    close(a2d[0]);
    close(b2d[0]);
    close(c2d[0]);
    close(d2b[1]);
  }

  return 0;
}
