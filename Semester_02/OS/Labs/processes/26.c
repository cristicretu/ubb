#include <ctype.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

int main(int argc, char** argv) {
  if (argc != 2) {
    printf("Usage: ./a.out <filename>\n");
    exit(1);
  }

  int p2c[2], c2p[2];

  if (-1 == pipe(p2c) || -1 == pipe(c2p)) {
    perror("pipe");
    exit(1);
  }

  pid_t pid = fork();
  if (pid < 0) {
    perror("fork");
    exit(1);
  } else if (pid == 0) {
    // child
    close(c2p[0]);
    int fd = fopen(argv[1], "r");
    if (fd < 0) {
      perror("Could not open file\n");
      exit(1);
    }

    int k;
    char* buf = malloc(1024);
    bool found = 0;

    while ((k = read(fd, buf, 1024)) > 0) {
      for (int i = 0; i < k; ++i) {
        if (buf[i] == '.') {
          found = 1;
        } else if (found) {
          buf[i] = tolower(buf[i]);
          found = 0;
        }
      }

      if (write(c2p[1], &k, sizeof(k)) < 0) {
        perror("write");
        exit(1);
      }

      if (write(c2p[1], buf, k) < 0) {
        perror("write");
        exit(1);
      }

      free(buf);
      close(fd);
      close(c2p[1]);
    }

  } else {
    // parent

    close(c2p[1]);
    int k;
    char* buf = malloc(1024);

    // read the length of the string
    while (read(c2p[0], &k, sizeof(k)) > 0) {
      // read the string
      if (read(c2p[0], buf, k) < 0) {
        perror("read");
        exit(1);
      }

      for (int i = 0; i < k; ++i) {
        printf("%c", buf[i]);
      }
    }

    free(buf);
    close(c2p[0]);

    wait(NULL);
  }
  return 0;
}
