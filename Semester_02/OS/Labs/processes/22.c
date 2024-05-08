#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/wait.h>
#include <pthread.h>
#include "pthread_barrier.h"
#include <semaphore.h>
#include <semaphore.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/time.h>

#define CHILD_COUNT 10 
#define SEM_NAME "/mysemaphore"

int main(int argc, char **argv) {
    /* sem_t *sem = mmap(NULL, sizeof(sem_t), PROT_READ | PROT_WRITE, MAP_SHARED | MAP_ANONYMOUS, -1, 0); */
    sem_t *sem = sem_open(SEM_NAME, O_CREAT, 0660, 0);
    if (sem == SEM_FAILED) {
        perror("sem_open");
        exit(EXIT_FAILURE);
    }

    struct timeval start_time, end_time;
    gettimeofday(&start_time, NULL);

    for (int i = 0; i < CHILD_COUNT; ++i) {
        pid_t pid = fork();
        if (pid < 0) {
            perror("error on fork");
            exit(1);
        } else if (pid == 0) { // child

            sem_wait(sem);
            usleep(1000 * 1000);

            printf("child %i has finished \n", i);
            sem_post(sem);
            exit(0);

        } 
    }

    sem_post(sem); /// start the first child
    for (int i = 0; i < CHILD_COUNT; ++i) {
        wait(NULL); /// Wait for each child to complete
    }

    gettimeofday(&end_time, NULL);
    long elapsed_time = (end_time.tv_sec - start_time.tv_sec) * 1000 + (end_time.tv_usec - start_time.tv_usec) / 1000;
    printf("Elapsed time: %ld ms\n", elapsed_time);

    sem_close(sem);
    sem_unlink(SEM_NAME);
    return 0;
}
