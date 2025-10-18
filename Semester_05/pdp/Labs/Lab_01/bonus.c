#include <pthread.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>

#define NUM_THREADS 8
#define OPERATIONS_PER_THREAD 50

typedef struct node {
  int value;
  struct node *next;
  struct node *prev;
  pthread_mutex_t mtx;
} node_t;

node_t *global_head = NULL;
pthread_mutex_t mutex_head;

node_t *create_node(node_t *node, int value) {
  node = malloc(sizeof(node_t));
  node->value = value;
  node->next = NULL;
  node->prev = NULL;
  pthread_mutex_init(&node->mtx, NULL);
  return node;
}

void lock_node(node_t *node) { pthread_mutex_lock(&node->mtx); }

void unlock_node(node_t *node) { pthread_mutex_unlock(&node->mtx); }

node_t *move_next(node_t *node) {
  lock_node(node);
  node_t *next = node->next;
  unlock_node(node);
  return next;
}

node_t *move_prev(node_t *node) {
  lock_node(node);
  node_t *prev = node->prev;
  unlock_node(node);
  return prev;
}

node_t *insert_after(node_t *node, int value) {
  if (node == NULL)
    return NULL;

  node_t *new_node = create_node(NULL, value);

  lock_node(node);
  node_t *next = node->next;

  if (next != NULL) {
    lock_node(next);

    if (node->next == next && next->prev == node) {
      new_node->next = next;
      new_node->prev = node;
      next->prev = new_node;
      node->next = new_node;

      unlock_node(next);
      unlock_node(node);
    } else {
      unlock_node(next);
      unlock_node(node);
      free(new_node);
      return insert_after(node, value);
    }
  } else {
    if (node->next == NULL) {
      new_node->next = NULL;
      new_node->prev = node;
      node->next = new_node;
      unlock_node(node);
    }
  }

  return new_node;
}

node_t *insert_before(node_t *node, int value) {
  if (node == NULL)
    return NULL;

  node_t *new_node = create_node(NULL, value);

  lock_node(node);
  node_t *prev = node->prev;
  unlock_node(node);

  if (prev != NULL) {
    lock_node(prev);
    lock_node(node);

    if (prev->next == node && node->prev == prev) {
      new_node->next = node;
      new_node->prev = prev;
      prev->next = new_node;
      node->prev = new_node;

      unlock_node(node);
      unlock_node(prev);
    } else {
      unlock_node(node);
      unlock_node(prev);
      free(new_node);
      return insert_before(node, value);
    }
  } else {
    lock_node(node);
    if (node->prev == NULL) {
      new_node->next = node;
      new_node->prev = NULL;
      node->prev = new_node;

      pthread_mutex_lock(&mutex_head);
      if (global_head == node) {
        global_head = new_node;
      }
      pthread_mutex_unlock(&mutex_head);

      unlock_node(node);
    }
  }

  return new_node;
}
void free_node(node_t *node) {
  pthread_mutex_destroy(&node->mtx);
  free(node);
}

node_t *get_random_node() {
  pthread_mutex_lock(&mutex_head);
  node_t *head = global_head;
  pthread_mutex_unlock(&mutex_head);

  int count = 0;
  for (node_t *current = head; current != NULL; current = move_next(current)) {
    count++;
  }
  if (count == 0)
    return NULL;

  int target = rand() % count;
  node_t *current = head;
  for (int i = 0; i < target && current != NULL; i++) {
    current = move_next(current);
  }
  return current;
}

void *func(void *arg) {
  int thread_id = *(int *)arg;
  unsigned int seed = time(NULL) + thread_id;

  for (int i = 0; i < OPERATIONS_PER_THREAD; i++) {
    int operation = rand_r(&seed) % 4;
    node_t *target = get_random_node();

    if (target == NULL)
      continue;

    switch (operation) {
    case 0:
      insert_after(target, rand_r(&seed) % 1000);
      break;
    case 1:
      insert_before(target, rand_r(&seed) % 1000);
      break;
    case 2:
      move_next(target);
      break;
    case 3:
      move_prev(target);
      break;
    }
  }
  return NULL;
}

bool validate_list() {
  pthread_mutex_lock(&mutex_head);
  node_t *head = global_head;
  pthread_mutex_unlock(&mutex_head);

  int next = 0, back = 0;
  node_t *tail = head;

  for (node_t *c = head; c != NULL; c = move_next(c)) {
    next++;
    tail = c;
    node_t *next = move_next(c);
    if (next && move_prev(next) != c)
      return false;
  }

  for (node_t *c = tail; c != NULL; c = move_prev(c)) {
    back++;
  }

  return next == back;
}

int main(int argc, char *argv[]) {
  srand(time(NULL));
  pthread_mutex_init(&mutex_head, NULL);

  global_head = create_node(NULL, 0);
  node_t *current = global_head;
  for (int i = 1; i < 100; i++) {
    current = insert_after(current, i);
  }

  pthread_mutex_lock(&mutex_head);
  node_t *head = global_head;
  pthread_mutex_unlock(&mutex_head);

  for (node_t *c = head; c != NULL; c = move_next(c)) {
    printf("%d ", c->value);
  }
  printf("\n");

  pthread_t threads[NUM_THREADS];
  int thread_ids[NUM_THREADS];

  for (int i = 0; i < NUM_THREADS; i++) {
    thread_ids[i] = i;
    pthread_create(&threads[i], NULL, func, &thread_ids[i]);
  }

  for (int i = 0; i < NUM_THREADS; i++) {
    pthread_join(threads[i], NULL);
  }

  if (validate_list()) {
    printf("bine bos");
  } else {
    printf("nu e bine");
  }

  return 0;
}