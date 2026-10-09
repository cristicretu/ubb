#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/wait.h>
#include <time.h>

int is_boltz(int n) {
    if (n % 7 == 0) {
        return 1;
    }

    while (n) {
        if (n % 10 == 7) {
            return 1;
        }
        n /= 10;
    }
    return 0;
}

int main(int argc, char **argv) {
    if (argc != 2) {
        perror("bad!");
        exit(1);
    }

    int n = atoi(argv[1]);

    if (n <= 0) {
        perror("bad number");
        exit(1);
    }

    int p[n][2];

    for (int i = 0; i < n; ++i) {
        if (pipe(p[i]) == -1) {
            perror("on pipe creation");
            exit(1);
        }
    }

    srand(time(NULL)); 

    for (int i = 0; i < n; ++i) {
        pid_t f = fork();

        if (f < 0) {
            perror("on fork");
            exit(1);
        } else if (f == 0) {
            // close all unused pipes
            // don't close:
            // p[i][0]
            // p[next][1]

            int next = (i == n - 1) ? 0 : i + 1;

            for (int j = 0; j < n; ++j) {
                if (j != i) close(p[j][0]); // reading ones
                if (j != next) close(p[j][1]); // writing ones
            }

            while (1) {
                int num;
                if (read(p[i][0], &num, sizeof(int)) == -1) {
                    perror("on read");
                    exit(1);
                }

                int rip_game = (num == -1);
                if (num != -1) {
                    ++num;
                    if (is_boltz(num)) {
                        int chance = rand() % 3;
                        if (chance == 0) {
                            rip_game = 1;
                        } else {
                            printf("Process %d: boltz\n", i + 1);
                        }
                    } else {
                        printf("Process %d: %d\n", i + 1, num);
                    }
                }

                num = rip_game == 1 ? -1 : num;
                /* printf("next is %d, and rip_game is %d", next, rip_game); */
                if (write(p[next][1], &num, sizeof(int)) == -1) {
                    perror("on write to next");
                    close(p[i][0]), close(p[next][1]);
                    exit(1);
                }
                /* printf("---- i wrote to the next the number %d\n", num); */

                if (rip_game == 1) {
                    close(p[i][0]), close(p[next][1]);
                    exit(0);
                }
            }
        }
    }

    for (int i = 1; i < n; ++i) {
        close(p[i][0]), close(p[i][1]);
    }
    close(p[0][0]);

    int num = 1;
    if (write(p[0][1], &num, sizeof(int)) == -1) {
        perror("on write first number");
        exit(1);
    }
    close(p[0][1]);

    for (int i = 0; i < n; ++i) {
        wait(NULL); 
    }

    return 0;
}
