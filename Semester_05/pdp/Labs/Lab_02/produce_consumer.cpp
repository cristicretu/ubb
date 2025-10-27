#include <chrono>
#include <condition_variable>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <mutex>
#include <pthread.h>
#include <queue>
#include <vector>

int VECTOR_SIZE = 101;
int QUEUE_MAX_SIZE = 100;
int MAX_VALUE = 5;
int MIN_VALUE = 1;

std::vector<int> vector1;
std::vector<int> vector2;
std::queue<int> queue;
std::mutex mutex;
std::condition_variable full_q;
std::condition_variable empty_q;

pthread_t producer_thread;
pthread_t consumer_thread;

void *producer(void *arg) {
  for (int i = 0; i < VECTOR_SIZE; ++i) {
    int pi = vector1[i] * vector2[i];
    {
      std::unique_lock<std::mutex> lock(mutex);
      while (queue.size() >= QUEUE_MAX_SIZE) {
        full_q.wait(lock);
      }
      queue.push(pi);
      empty_q.notify_one();
    }
  }
  return NULL;
}

void *consumer(void *arg) {
  int sum = 0;
  for (int i = 0; i < VECTOR_SIZE; ++i) {
    {
      std::unique_lock<std::mutex> lock(mutex);
      while (queue.empty()) {
        empty_q.wait(lock);
      }
      int pi = queue.front();
      queue.pop();
      sum += pi;
      full_q.notify_one();
    }
  }
  std::cout << "suma finala: " << sum << std::endl;
  return NULL;
}

int main() {
  srand(static_cast<unsigned>(time(NULL)));
  vector1.resize(VECTOR_SIZE);
  vector2.resize(VECTOR_SIZE);
  for (int i = 0; i < VECTOR_SIZE; ++i) {
    // vector1[i] = rand() % (MAX_VALUE - MIN_VALUE + 1) + MIN_VALUE;
    // vector2[i] = rand() % (MAX_VALUE - MIN_VALUE + 1) + MIN_VALUE;
    vector1[i] = 1;
    vector2[i] = i;
    std::cout << "vector1[" << i << "] = " << vector1[i] << ", vector2[" << i
              << "] = " << vector2[i] << std::endl;
  }

  auto start = std::chrono::high_resolution_clock::now();

  pthread_create(&producer_thread, NULL, producer, NULL);
  pthread_create(&consumer_thread, NULL, consumer, NULL);

  pthread_join(producer_thread, NULL);
  pthread_join(consumer_thread, NULL);

  auto end = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = end - start;
  std::cout << elapsed.count() * 1000 << "s" << std::endl;

  return 0;
}
