#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <ctype.h>
#include <string.h>

char *fifo1 = "./fifo1";
char *fifo2 = "./fifo2";

int main(int argc, char **argv) {
    if (0 > mkfifo(fifo1, 0666)) {
        perror("fifo1");
        exit(1);
    }
    if (0 > mkfifo(fifo2, 0666)) {
        perror("fifo2");
        exit(1);
    }
    int fd_write = open(fifo1, O_WRONLY);
    if (-1 == fd_write) {
        perror("fifo1");
        exit(1);
    }

    int fd_read = open(fifo2, O_RDONLY);
    if (-1 == fd_read) {
        perror("fifo2");
        exit(1);
    }

    int size = 0;
    for (int i = 1; i < argc; ++i) {
        size += strlen(argv[i]) + 1;
    }
    ++size;

    int i = 1;
    char *ans = (char *)malloc(size * sizeof(char));
    while (1) {
        if (i > argc) {
            break;
        }

        int k = strlen(argv[i]);
        if (-1 == write(fd_write, k, sizeof(int))) {
            perror("write len");
            close(fd_write);
            close(fd_read);
            exit(1);
        }

        if (-1 == write(fd_write, argv[i], k * sizeof(char))) {
            perror("write str");
            close(fd_write);
            close(fd_read);
            exit(1);
        }

        int len;
        if (-1 == read(fd_read, &len, sizeof(int))) {
            perror("len");
            exit(1);
        }

        char *buf = malloc((len + 1) * sizeof(char));

        int r = 0;
        while (read < len + 1) {
            int k;

            if ((k = read(fd_read, buf + r, (len - r) * sizeof(char))) > 0) {
                r += k;
            }
        }

        buf[len] = 0;
        strcat(ans, buf);
        strcat(ans, " ");
        free(buf);
    }

    free(ans);
    close(fd_write);
    close(fd_read);
    if (0 > unlink(fifo1)) {
        perror("error unlink");
    }
    if (0 > unlink(fifo2)) {
        perror("error unlink");
    }
    return 0;
}
