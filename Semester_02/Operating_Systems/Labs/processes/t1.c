/*
 * Problem: for each command line arg create 2 child processes A and B. A will
 * compute the length of its arg and send it to parent using its own
 * communication channel. B will compute the sum of all digits in the arg and
 * send it to the parent using its own communication channel. After all hild
 * processes are finished, compute in the parent the average of the results from
 * processes A and the sum of rez of child processes B and print them
 *
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

int main(int argc, char **argv) {
  if (argc < 2) {
    perror("incorrect usage");
    exit(1);
  }

  int a2p[argc - 1][2];
  int b2p[argc - 1][2];

  for (int i = 0; i < argc - 1; ++i) {
    if (-1 == pipe(a2p[i]) || -1 == pipe(b2p[i])) {
      perror("on pipe");
      exit(1);
    }
  }

  for (int i = 0; i < argc - 1; ++i) {
    pid_t a = fork();

    if (a < 0) {
      perror("on a creation");
      exit(1);
    } else if (a == 0) {
      //   for (int j = 0; j < argc; ++j) {
      //     if (j != i) close(a2p[i][1]);
      //     close(a2p[i][0]), close(b2p[i][0]), close(b2p[i][1]);
      //   }
      for (int j = 0; j < argc - 1; ++j) {
        if (j != i) {
          close(a2p[j][0]);
          close(a2p[j][1]);
          close(b2p[j][0]);
          close(b2p[j][1]);
        }
      }
      close(a2p[i][0]);
      int len = strlen(argv[i + 1]);

      if (-1 == write(a2p[i][1], &len, sizeof(int))) {
        perror("on write to parent");
        close(a2p[i][1]);
        exit(1);
      }
      close(a2p[i][1]);
      exit(1);
    }

    pid_t b = fork();

    if (b < 0) {
      perror("on b creation");
      exit(1);
    } else if (b == 0) {
      //   for (int j = 0; j < argc; ++j) {
      //     if (j != i) close(b2p[i][1]);
      //     close(b2p[i][0]), close(a2p[i][0]), close(a2p[i][1]);
      //   }
      for (int j = 0; j < argc - 1; ++j) {
        if (j != i) {
          close(a2p[j][0]);
          close(a2p[j][1]);
          close(b2p[j][0]);
          close(b2p[j][1]);
        }
      }
      close(b2p[i][0]);
      int sum = 0;
      for (int j = 0, n = strlen(argv[i + 1]); j < n; ++j) {
        if (argv[i + 1][j] >= '0' && argv[i + 1][j] <= '9') {
          sum += argv[i + 1][j] - '0';
        }
      }

      if (-1 == write(b2p[i][1], &sum, sizeof(int))) {
        perror("on write b to parent");
        close(b2p[i][1]);
        exit(1);
      }
      close(b2p[i][1]);
      exit(1);
    }
  }

  int total_len = 0, total_sum = 0;

  for (int i = 0; i < argc - 1; ++i) {
    int len, sum;
    close(a2p[i][1]);
    close(b2p[i][1]);

    if (-1 == read(a2p[i][0], &len, sizeof(int))) {
      perror("on read from a child");
      exit(1);
    }
    close(a2p[i][0]);
    total_len += len;

    if (-1 == read(b2p[i][0], &sum, sizeof(int))) {
      perror("on read from b child");
      exit(1);
    }
    close(b2p[i][0]);
    total_sum += sum;
  }

  printf("Average length of the arguments is %f\n",
         (float)total_len / (argc - 1) * 1.0);
  printf("The sum of all digits from the arguments is %d\n", total_sum);

  for (int i = 0; i < 2 * (argc - 1); ++i) {
    wait(NULL);
  }
  return 0;
}
