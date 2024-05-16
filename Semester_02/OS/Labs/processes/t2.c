#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

int is_prime(int n) {
  if (n < 2) return 0;
  if (n == 2) return 1;

  for (int d = 2; d * d <= n; ++d) {
    if (n % d == 0) return 0;
  }

  return 1;
}

int main(int argc, char **argv) {
  int a2b[2], b2a[2];

  if (-1 == pipe(a2b)) {
    perror("pipe");
    exit(1);
  }

  if (-1 == pipe(b2a)) {
    perror("pipe");
    close(a2b[0]), close(a2b[1]);
    exit(1);
  }

  pid_t a = fork();
  if (a < 0) {
    perror("fork");
    close(a2b[0]), close(a2b[1]);
    close(b2a[0]), close(b2a[1]);
    exit(1);
  } else if (a == 0) {
    close(b2a[1]);
    close(a2b[0]);

    int n;

    while (1) {
      scanf("%d", &n);

      int res = is_prime(n);
      n = res ? -1 : n;
      if (-1 == write(a2b[1], &n, sizeof(n))) {
        perror("write");
        close(a2b[1]);
        close(b2a[0]);
        exit(1);
      }

      if (res) break;

      int len;
      if (-1 == read(b2a[0], &len, sizeof(int))) {
        perror("read");
        close(a2b[1]);
        close(b2a[0]);
        exit(1);
      }

      int *arr = (int *)malloc(len * sizeof(int));
      if (-1 == read(b2a[0], arr, len * sizeof(int))) {
        perror("read");
        close(a2b[1]);
        close(b2a[0]);
        exit(1);
      }

      printf("Number of divisors: %d\n", len);
      printf("Divisors: ");
      for (int i = 0; i < len; ++i) {
        printf("%d ", arr[i]);
      }
      printf("\n");

      free(arr);
    }

    close(a2b[1]);
    close(b2a[0]);
    exit(0);
  }

  pid_t b = fork();
  if (b < 0) {
    perror("fork");
    close(a2b[0]), close(a2b[1]);
    close(b2a[0]), close(b2a[1]);
    exit(1);
  } else if (b == 0) {
    close(a2b[1]);
    close(b2a[0]);

    int n;

    while (1) {
      if (-1 == read(a2b[0], &n, sizeof(n))) {
        perror("read on b");
        close(a2b[0]);
        close(b2a[1]);
        exit(1);
      }

      if (n == -1) break;

      int len = 0;
      for (int i = 2; i < n; ++i) {
        if (n % i == 0) {
          ++len;
        }
      }

      if (-1 == write(b2a[1], &len, sizeof(int))) {
        perror("write");
        close(a2b[0]);
        close(b2a[1]);
        exit(1);
      }

      int *arr = (int *)malloc(len * sizeof(int));
      int j = 0;
      for (int i = 2; i < n; ++i) {
        if (n % i == 0) {
          arr[j++] = i;
        }
      }

      if (-1 == write(b2a[1], arr, len * sizeof(int))) {
        perror("write");
        close(a2b[0]);
        close(b2a[1]);
        exit(1);
      }
    }

    close(a2b[0]);
    close(b2a[1]);
    exit(0);
  }

  close(a2b[0]);
  close(a2b[1]);
  close(b2a[0]);
  close(b2a[1]);

  wait(NULL);
  wait(NULL);
  return 0;
}
