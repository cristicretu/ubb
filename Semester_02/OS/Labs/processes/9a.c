#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>

char *fifo1 = "./fifo1";
char *fifo2 = "./fifo2";

int main(int argc, char ** argv) {
    if (-1 == mkfifo(fifo1, 0666)) {
        perror("error on mkfifo1");
        exit(1);
    }

    if (-1 == mkfifo(fifo2, 0666)) {
        perror("error on mkfifo2");
        exit(1);
    }

    int fd_write = open(fifo1, O_WRONLY);
    if (-1 == fd_write) {
        perror("rip fd write");
        exit(1);
    }

    int fd_read = open(fifo2, O_RDONLY);
    if (-1 == fd_read) {
        perror("fd read");
        exit(1);
    }

    while (1) {
        if ((k = read(0, )))
    }

    if (-1 == unlink(fifo1)) {
        perror("unlink fifo1");
        exit(1);
    }

    if (-1 == unlink(fifo2)) {
        perror("unlink fifo2");
        exit(1);
    }
    return 0;
}

