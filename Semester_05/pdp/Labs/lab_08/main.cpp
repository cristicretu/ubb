#include "dsm.hpp"
#include <cstdio>
#include <cstdlib>
#include <sys/wait.h>

void run(int rank, int n) {
  DSM dsm(rank, n);

  std::set<int> all;
  for (int i = 0; i < n; i++)
    all.insert(i);
  dsm.subscribe(0, all, 0);
  dsm.subscribe(1, {0, 1}, 0);

  dsm.set_callback([&](int var, int oldv, int newv) {
    printf("[P%d] var[%d]: %d -> %d\n", rank, var, oldv, newv);
  });

  dsm.start();
  dsm.sync();

  printf("[P%d] === writes ===\n", rank);
  if (rank == 0)
    dsm.write(0, 42);
  dsm.sync();
  printf("[P%d] var[0] = %d\n", rank, dsm.read(0));
  dsm.sync();

  printf("[P%d] === ordering ===\n", rank);
  for (int i = 0; i < n; i++) {
    if (rank == i) {
      dsm.write(0, rank * 10 + 1);
      dsm.write(0, rank * 10 + 2);
    }
    dsm.sync();
  }

  printf("[P%d] === cas ===\n", rank);
  if (rank == 0)
    dsm.write(0, 0);
  dsm.sync();
  int old;
  bool ok = dsm.cas(0, 0, rank + 100, &old);
  printf("[P%d] cas(0, 0, %d) = %s, old=%d\n", rank, rank + 100,
         ok ? "ok" : "fail", old);
  dsm.sync();

  printf("[P%d] === counter ===\n", rank);
  if (rank == 0)
    dsm.write(0, 0);
  dsm.sync();
  for (int i = 0; i < 3; i++) {
    while (true) {
      int cur = dsm.read(0);
      if (dsm.cas(0, cur, cur + 1))
        break;
    }
  }
  dsm.sync();
  dsm.sync();
  printf("[P%d] final = %d (expect %d)\n", rank, dsm.read(0), n * 3);

  dsm.stop();
}

int main(int argc, char **argv) {
  int n = argc > 1 ? atoi(argv[1]) : 3;
  printf("spawning %d processes\n", n);

  std::vector<pid_t> pids;
  for (int i = 0; i < n; i++) {
    pid_t p = fork();
    if (p == 0) {
      run(i, n);
      exit(0);
    }
    pids.push_back(p);
  }
  for (pid_t p : pids)
    waitpid(p, 0, 0);
  printf("done\n");
}
