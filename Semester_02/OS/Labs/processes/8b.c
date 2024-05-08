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
    int fd_read = open(fifo1, O_RDONLY);
    if (-1 == fd_read) {
        perror("fifo1");
        exit(1);
    }

    int fd_write= open(fifo2, O_WRONLY);
    if (-1 == fd_write) {
        perror("fifo2");
        exit(1);
    }

    while (1) {
        int len;
        if (-1 == read(fd_read, &len, sizeof(int))) {
            perror("len");
            exit(1);
        }

        if (len < 0) {
            break;
        }

        char *buf = malloc((len + 1) * sizeof(char));

        int r = 0;
        while (read < len + 1) {
            int k;

            if ((k = read(fd_read, buf + r, (len - r) * sizeof(char))) > 0) {
                r += k;
            }
        }

        for (int i = 0; i < len; ++i) {
            if(buf[i] >= 'a' && buf[i] <= 'z') {
                buf[i] += 'A' - 'a';
            }
        }


        int k = len;
        if (-1 == write(fd_write, k, sizeof(int))) {
            perror("write len");
            close(fd_write);
            close(fd_read);
            exit(1);
        }

        if (-1 == write(fd_write, buf, k * sizeof(char))) {
            perror("write str");
            close(fd_write);
            close(fd_read);
            exit(1);
        }


        free(buf);
    }

    close(fd_write);
    close(fd_read);
    return 0;
}
