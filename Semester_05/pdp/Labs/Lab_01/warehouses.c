/*
Common requirements
The problems will require to execute a number of independent operations, that operate on shared data.
There shall be several threads launched at the beginning, and each thread shall execute a lot of operations. The operations to be executed are to be randomly choosen, and with randomly choosen parameters.
The main thread shall wait for all other threads to end and, then, it shall check that the invariants are obeyed.
The operations must be synchronized in order to operate correctly. Write, in a documentation, the rules (which mutex what invariants it protects).
You shall play with the number of threads and with the granularity of the locking, in order to asses the performance issues. Document what tests have you done, on what hardware platform, for what size of the data, and what was the time consumed.
Problems
1. Warehouses:
A wholesaler has several warehouses storing goods.

We must keep track of the quantity of each product, in each of the warehouses.

We have some moves between warehouses running concurrently, on several threads. Each move consists in moving some given amounts of some product types from a given source warehouse to another, given, destination warehouse.

From time to time, as well as at the end, an inventory check operation shall be run. It shall check that, for each product type, the total amount of that product in all warehouses is the same as in the beginning.

Two moves involving distinct warehouses, or involving disjoint sets of products, must be able to be processed independently (without having to wait for the same mutex).

*/
#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <time.h>
#include <unistd.h>
#include <string.h>

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


pthread_mutex_t mutex;
const int num_threads = 4;
const int num_warehouses = 4;
const int num_products = 2;

const char *product_names[] = {
  "product1",
  "product2",
  "product3",
  "product4",
  "product5",
};

const char *warehouse_names[] = {
  "warehouse1",
  "warehouse2",
  "warehouse3",
  "warehouse4",
  "warehouse5",
};

struct product {
  char name[100];
  int quantity;
  pthread_mutex_t mutex;
};

struct warehouse {
  int id;
  struct product products[num_products];
};

void *worker_func(void *arg) {
  int id = (int)arg;
  printf("Thread %d started\n", id);
  return NULL;
}

void *checker_func(void *arg) {
  printf("Checker started\n");
  return NULL;
}

int main(int argc, char *argv[]) {
  srand((unsigned int)time(NULL));
  struct warehouse warehouses[num_warehouses];
  for (int i = 0; i < num_warehouses; i++) {
    warehouses[i].id = i;
    const char *random_warehouse_name = warehouse_names[rand() % (sizeof(warehouse_names)/sizeof(warehouse_names[0]))];
    for (int j = 0; j < num_products; j++) {
      const char *random_product_name = product_names[rand() % (sizeof(product_names)/sizeof(product_names[0]))];
      strncpy(warehouses[i].products[j].name, random_product_name, sizeof(warehouses[i].products[j].name) - 1);
      warehouses[i].products[j].name[sizeof(warehouses[i].products[j].name) - 1] = '\0';
      warehouses[i].products[j].quantity = rand() % 100;
    }
  }

  printf("--------------------------------\n");
  for (int i = 0; i < num_warehouses; i++) {
    printf("Warehouse %d: %s\n", warehouses[i].id, warehouse_names[i]);
    for (int j = 0; j < num_products; j++) {
      printf("  Product %d: %s, %d\n", j, warehouses[i].products[j].name, warehouses[i].products[j].quantity);
    }
  }
  printf("--------------------------------\n");

  pthread_t threads[num_threads];
  for (int i = 0; i < num_threads; i++) {
    pthread_create(&threads[i], NULL, worker_func, (void *)&i);
  }
  for (int i = 0; i < num_threads; i++) {
    pthread_join(threads[i], NULL);
  }

  pthread_t checker_thread;
  pthread_create(&checker_thread, NULL, checker_func, NULL);
  pthread_join(checker_thread, NULL);

  return 0;
}