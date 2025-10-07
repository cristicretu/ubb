/*
Common requirements
The problems will require to execute a number of independent operations, that
operate on shared data. There shall be several threads launched at the
beginning, and each thread shall execute a lot of operations. The operations to
be executed are to be randomly choosen, and with randomly choosen parameters.
The main thread shall wait for all other threads to end and, then, it shall
check that the invariants are obeyed. The operations must be synchronized in
order to operate correctly. Write, in a documentation, the rules (which mutex
what invariants it protects). You shall play with the number of threads and with
the granularity of the locking, in order to asses the performance issues.
Document what tests have you done, on what hardware platform, for what size of
the data, and what was the time consumed. Problems
1. Warehouses:
A wholesaler has several warehouses storing goods.

We must keep track of the quantity of each product, in each of the warehouses.

We have some moves between warehouses running concurrently, on several threads.
Each move consists in moving some given amounts of some product types from a
given source warehouse to another, given, destination warehouse.

From time to time, as well as at the end, an inventory check operation shall be
run. It shall check that, for each product type, the total amount of that
product in all warehouses is the same as in the beginning.

Two moves involving distinct warehouses, or involving disjoint sets of products,
must be able to be processed independently (without having to wait for the same
mutex).

*/
#include <limits.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>

#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))

/*
operation:
- choose list of random prod, qty
*/

/*
partial order relation:
- lock products from warehouse with id '<' first
- lock products from warehouse with id '>' second
- transfer products
- unlock in reverse order
*/

const int num_threads = 16;
const int num_warehouses = 10;
const int num_products = 200;

volatile int running = 1;
int total_ops = 0;
const int max_ops = 1e6;
pthread_mutex_t total_ops_mutex;

const char *product_names[] = {
    "product1", "product2", "product3", "product4", "product5",
    "product6", "product7", "product8", "product9", "product10",
};

const char *warehouse_names[] = {
    "warehouse1", "warehouse2", "warehouse3", "warehouse4", "warehouse5",
    "warehouse6", "warehouse7", "warehouse8", "warehouse9", "warehouse10",
};

struct product {
  char name[100];
  int quantity;
  pthread_mutex_t futex;
};

struct warehouse {
  int id;
  struct product products[num_products];
};

struct thread_args {
  int thread_id;
  struct warehouse *warehouses;
};

struct checker_args {
  struct warehouse *warehouses;
  int *initial_totals;
};

void transfer(struct warehouse warehouses[], int from, int to, int products[],
              int quantity[], int num_items) {
  for (int i = 0; i < num_items; i++) {
    warehouses[from].products[products[i]].quantity -= quantity[i];
    warehouses[to].products[products[i]].quantity += quantity[i];
  }
}

void *worker_func(void *arg) {
  struct thread_args *args = (struct thread_args *)arg;
  int id = args->thread_id;
  struct warehouse *warehouses = args->warehouses;
  printf("thread %d started\n", id);
  while (running) {
    int from = rand() % num_warehouses;
    int to = rand() % num_warehouses;

    int de_mutat = 1 + rand() % num_products;
    int products[de_mutat];
    int qty[de_mutat];

    int used[num_products];
    for (int i = 0; i < num_products; i++)
      used[i] = 0;

    int count = 0;
    while (count < de_mutat) {
      int prod = rand() % num_products;
      if (!used[prod]) {
        products[count] = prod;
        used[prod] = 1;
        count++;
      }
    }

    for (int i = 0; i < de_mutat - 1; i++) {
      for (int j = i + 1; j < de_mutat; j++) {
        if (products[i] > products[j]) {
          int temp = products[i];
          products[i] = products[j];
          products[j] = temp;
        }
      }
    }

    if (from != to) {
      int first = MIN(from, to);
      int second = MAX(from, to);

      for (int i = 0; i < de_mutat; i++) {
        pthread_mutex_lock(&warehouses[first].products[products[i]].futex);
      }
      for (int i = 0; i < de_mutat; i++) {
        pthread_mutex_lock(&warehouses[second].products[products[i]].futex);
      }

      for (int i = 0; i < de_mutat; i++) {
        int available = warehouses[from].products[products[i]].quantity;
        qty[i] = rand() % (available + 1);
      }

      transfer(warehouses, from, to, products, qty, de_mutat);

      for (int i = de_mutat - 1; i >= 0; i--) {
        pthread_mutex_unlock(&warehouses[second].products[products[i]].futex);
      }
      for (int i = de_mutat - 1; i >= 0; i--) {
        pthread_mutex_unlock(&warehouses[first].products[products[i]].futex);
      }
    }

    pthread_mutex_lock(&total_ops_mutex);
    total_ops++;
    if (total_ops >= max_ops) {
      running = 0;
      pthread_mutex_unlock(&total_ops_mutex);
      break;
    }
    pthread_mutex_unlock(&total_ops_mutex);
  }
  return NULL;
}

void check_invariants(struct warehouse warehouses[], int initial_totals[]) {
  for (int i = 0; i < num_warehouses; i++) {
    for (int j = 0; j < num_products; j++) {
      pthread_mutex_lock(&warehouses[i].products[j].futex);
    }
  }

  for (int p = 0; p < num_products; p++) {
    int total = 0;
    for (int w = 0; w < num_warehouses; w++) {
      total += warehouses[w].products[p].quantity;
    }
    if (total != initial_totals[p]) {
      printf(
          "Invariant violated: product %d total quantity is %d, expected %d\n",
          p, total, initial_totals[p]);
      exit(1);
    }
  }

  for (int i = num_warehouses - 1; i >= 0; i--) {
    for (int j = num_products - 1; j >= 0; j--) {
      pthread_mutex_unlock(&warehouses[i].products[j].futex);
    }
  }
}

void *checker_func(void *arg) {
  struct checker_args *args = (struct checker_args *)arg;
  struct warehouse *warehouses = args->warehouses;
  int *initial_totals = args->initial_totals;
  printf("Checker started\n");
  while (running) {
    check_invariants(warehouses, initial_totals);
    sleep(2);
  }
  printf("Checker finished\n");
  return NULL;
}

int main(int argc, char *argv[]) {
  srand((unsigned int)time(NULL));
  pthread_mutex_init(&total_ops_mutex, NULL);
  struct warehouse warehouses[num_warehouses];
  for (int i = 0; i < num_warehouses; i++) {
    warehouses[i].id = i;
    const char *random_warehouse_name =
        warehouse_names[rand() %
                        (sizeof(warehouse_names) / sizeof(warehouse_names[0]))];
    for (int j = 0; j < num_products; j++) {
      const char *random_product_name =
          product_names[rand() %
                        (sizeof(product_names) / sizeof(product_names[0]))];
      strncpy(warehouses[i].products[j].name, random_product_name,
              sizeof(warehouses[i].products[j].name) - 1);
      warehouses[i]
          .products[j]
          .name[sizeof(warehouses[i].products[j].name) - 1] = '\0';
      warehouses[i].products[j].quantity = rand() % 100;
      pthread_mutex_init(&warehouses[i].products[j].futex, NULL);
    }
  }

  printf("--------------------------------\n");
  for (int i = 0; i < num_warehouses; i++) {
    printf("Warehouse %d: %s\n", warehouses[i].id, warehouse_names[i]);
    for (int j = 0; j < num_products; j++) {
      printf("  Product %d: %s, %d\n", j, warehouses[i].products[j].name,
             warehouses[i].products[j].quantity);
    }
  }
  printf("--------------------------------\n");

  int initial_totals[num_products];
  for (int i = 0; i < num_products; i++) {
    initial_totals[i] = 0;
    for (int j = 0; j < num_warehouses; j++) {
      initial_totals[i] += warehouses[j].products[i].quantity;
    }
  }

  printf("Initial totals: ");
  for (int i = 0; i < num_products; i++) {
    printf("%d ", initial_totals[i]);
  }
  printf("\n");

  struct timeval start_time, end_time;
  gettimeofday(&start_time, NULL);

  pthread_t threads[num_threads];
  struct thread_args args[num_threads];
  for (int i = 0; i < num_threads; i++) {
    args[i].thread_id = i;
    args[i].warehouses = warehouses;
    pthread_create(&threads[i], NULL, worker_func, (void *)&args[i]);
  }

  struct checker_args checker_args;
  checker_args.warehouses = warehouses;
  checker_args.initial_totals = initial_totals;

  pthread_t checker_thread;
  pthread_create(&checker_thread, NULL, checker_func, &checker_args);

  for (int i = 0; i < num_threads; i++) {
    pthread_join(threads[i], NULL);
  }

  pthread_join(checker_thread, NULL);

  gettimeofday(&end_time, NULL);

  printf("\nPerforming final invariant check...\n");
  check_invariants(warehouses, initial_totals);
  printf("Final invariant check passed!\n");

  double elapsed_time = (end_time.tv_sec - start_time.tv_sec) +
                        (end_time.tv_usec - start_time.tv_usec) / 1e6;

  printf("\n========================================\n");
  printf("Performance Statistics:\n");
  printf("========================================\n");
  printf("Number of threads:     %d\n", num_threads);
  printf("Number of warehouses:  %d\n", num_warehouses);
  printf("Number of products:    %d\n", num_products);
  printf("Number of operations:  %d\n", total_ops);
  printf("Time taken:            %.3f seconds\n", elapsed_time);
  printf("Operations per second: %.2f\n", total_ops / elapsed_time);
  printf("========================================\n");

  for (int i = 0; i < num_warehouses; i++) {
    for (int j = 0; j < num_products; j++) {
      pthread_mutex_destroy(&warehouses[i].products[j].futex);
    }
  }
  pthread_mutex_destroy(&total_ops_mutex);

  return 0;
}

/*
========================================
Performance Statistics:
========================================
Number of threads:     4
Number of warehouses:  4
Number of products:    2
Number of operations:  1000000
Time taken:            2.003 seconds
Operations per second: 499228.69
========================================

========================================
Performance Statistics:
========================================
Number of threads:     16
Number of warehouses:  4
Number of products:    2
Number of operations:  1000000
Time taken:            2.006 seconds
Operations per second: 498599.19
========================================


========================================
Performance Statistics:
========================================
Number of threads:     4
Number of warehouses:  5
Number of products:    100
Number of operations:  1000003
Time taken:            4.007 seconds
Operations per second: 249570.30
========================================

========================================
Performance Statistics:
========================================
Number of threads:     16
Number of warehouses:  5
Number of products:    100
Number of operations:  1000015
Time taken:            4.004 seconds
Operations per second: 249731.36
========================================

========================================
Performance Statistics:
========================================
Number of threads:     4
Number of warehouses:  10
Number of products:    200
Number of operations:  1000003
Time taken:            4.010 seconds
Operations per second: 249376.56
========================================

========================================
Performance Statistics:
========================================
Number of threads:     16
Number of warehouses:  10
Number of products:    200
Number of operations:  1000015
Time taken:            4.008 seconds
Operations per second: 249523.17
========================================
*/
