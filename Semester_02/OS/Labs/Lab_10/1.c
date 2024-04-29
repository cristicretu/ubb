#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

// creates a child and reads n integers from there, stores them in array, sends
// it back to parent via pipes parent displays the array

int main(int argc, char **argv) {
  if (argc != 2) {
    printf("yooooo input a filename bro!!!");
    return 1;
  }

  FILE *fd = fopen(argv[1], "r");
  if (fd == NULL) {
    printf("yooooo file not found bro!!!");
    return 1;
  }

  int n = 0;
  fscanf(fd, "%d", &n);

  if (n <= 0) {
    printf("yooo the number cannot be negative!!!");
    return 1;
  }

  pid_t pid;

  int c2p[2], p2c[2];

  if (pipe(c2p) == -1 || pipe(p2c) == -1) {
    printf("yooo pipe error!!!");
    return 1;
  }

  pid = fork();

  if (pid < 0) {
    printf("yooo fork error!!!");
    return 1;
  } else if (pid == 0) {
    // child
    close(c2p[0]);
    close(p2c[1]);

    int *arr = (int *)malloc(n * sizeof(int));
    for (int i = 0; i < n; i++) {
      fscanf(fd, "%d", &arr[i]);
    }

    write(c2p[1], arr, n * sizeof(int));

    close(c2p[1]);

    close(p2c[0]);
  } else {
    // parent
    close(c2p[1]);
    close(p2c[0]);

    int *arr = (int *)malloc(n * sizeof(int));
    read(c2p[0], arr, n * sizeof(int));

    printf("The number n is %d", n);

    int x = 0;
    int idx = 0;

    while (1) {
      printf("Read x:");
      scanf("%d", &x);

      if (x <= 0) {
        break;
      }

      if (idx + x > n) {
        x = n - idx;
      }

      for (int i = 0; i < x; i++) {
        printf("%d ", arr[idx + i]);
      }
      printf("\n");

      idx += x;

      if (idx >= n) {
        break;
      }

    }

    free(arr);

    close(c2p[0]);
    close(p2c[1]);
  }

  return 0;
}
