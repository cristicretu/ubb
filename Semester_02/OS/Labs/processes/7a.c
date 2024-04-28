// use fifo between two programs that send random numbers to one another, until one of them sends 10
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

char *fifo1 = "./fifo1";
char *fifo2 = "./fifo2";

int main() {
    if (0 > mkfifo(fifo1, 0666)) {
        perror("mkfifo");
        exit(1);
    }

    if (0 > mkfifo(fifo2, 0666)) {
        perror("mkfifo");
        exit(1);
    }

    int fd1 = open(fifo1, O_WRONLY);
    if (-1 == fd1) {
        perror("open");
        exit(1);
    }

    int fd2 = open(fifo2, O_RDONLY);
    if (-1 == fd2) {
        perror("open");
        exit(1);
    }

    srand(getpid());
    int num = 0;

    while (1) {
        num = rand() % 10;
        printf("Sending %d\n", num);
        if (-1 == write(fd1, &num, sizeof(num))) {
            perror("write");
            exit(1);
        }
        if (10 == num) {
            break;
        }

        if (-1 == read(fd2, &num, sizeof(num))) {
            perror("read");
            exit(1);
        }
        printf("Received %d\n", num);
        if (10 == num) {
            break;
        }
    }

    close(fd1);
    close(fd2);

    if (0 > unlink(fifo1)) {
        perror("unlink");
        exit(1);
    }

    if (0 > unlink(fifo2)) {
        perror("unlink");
        exit(1);
    }

    return 0;
}
