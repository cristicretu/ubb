#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <unistd.h>

#define ARRAY_SIZE 10

// Shared array
int shared_array[ARRAY_SIZE];
pthread_mutex_t lock;

typedef struct {
    int group_id;
    int thread_id;
    pthread_barrier_t *barrier;
} thread_data_t;

void *thread_func(void *arg) {
    thread_data_t *data = (thread_data_t *)arg;
    int group_id = data->group_id;
    int thread_id = data->thread_id;
    pthread_barrier_t *barrier = data->barrier;

    // Wait for all threads in the group to reach this point
    pthread_barrier_wait(barrier);

    // Generate a random number between 1 and 10
    int random_value = rand() % 10 + 1;

    // Multiply the 4th position of the shared array by the random value
    pthread_mutex_lock(&lock);
    shared_array[3] *= random_value;
    pthread_mutex_unlock(&lock);

    printf("Group %d, Thread %d, Random Value: %d, Shared Array[3]: %d\n", group_id, thread_id, random_value, shared_array[3]);

    return NULL;
}

int main(int argc, char *argv[]) {
    if (argc != 3) {
        fprintf(stderr, "Usage: %s <n> <m>\n", argv[0]);
        return 1;
    }

    int n = atoi(argv[1]);
    int m = atoi(argv[2]);
    pthread_t threads[n * m];
    pthread_barrier_t barriers[n];
    thread_data_t thread_data[n * m];

    // Initialize the shared array and mutex
    for (int i = 0; i < ARRAY_SIZE; i++) {
        shared_array[i] = 1;
    }
    pthread_mutex_init(&lock, NULL);

    // Initialize barriers
    for (int i = 0; i < n; i++) {
        pthread_barrier_init(&barriers[i], NULL, m);
    }

    // Create n * m threads
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < m; j++) {
            int idx = i * m + j;
            thread_data[idx].group_id = i;
            thread_data[idx].thread_id = j;
            thread_data[idx].barrier = &barriers[i];
            pthread_create(&threads[idx], NULL, thread_func, &thread_data[idx]);
        }
    }

    // Wait for all threads to complete
    for (int i = 0; i < n * m; i++) {
        pthread_join(threads[i], NULL);
    }

    // Find the index with the maximum value in the shared array
    int max_index = 0;
    int max_value = shared_array[0];
    for (int i = 1; i < ARRAY_SIZE; i++) {
        if (shared_array[i] > max_value) {
            max_value = shared_array[i];
            max_index = i;
        }
    }

    // Print the index with the maximum value and the maximum value
    printf("Index with maximum value: %d\n", max_index);
    printf("Maximum value: %d\n", max_value);

    // Destroy mutex and barriers
    pthread_mutex_destroy(&lock);
    for (int i = 0; i < n; i++) {
        pthread_barrier_destroy(&barriers[i]);
    }

    return 0;
}
