# Parallel and Distributed Programming

Semester 5 · Year 3 · C, C++, C#, Java, MPI, Metal

Threads, locks, condition variables, futures, thread pools, MPI and a bit of GPU. Most labs solve one problem several ways (sequential, threaded, distributed) and compare the running times.

## Contents

| Folder | What it is |
| --- | --- |
| [Labs/Lab_01](Labs/Lab_01) | Warehouses: concurrent stock moves between warehouses with fine-grained mutexes and inventory checks (`warehouses.c`). Bonus: doubly linked list with per-node locks (`bonus.c`) |
| [Labs/Lab_02](Labs/Lab_02) | Producer-consumer scalar product with a bounded queue and condition variables |
| [Labs/Lab_03](Labs/Lab_03) | Matrix multiplication with threads, split by rows, columns or every k-th element |
| [Labs/Lab_04](Labs/Lab_04) | C# HTTP downloader over raw sockets, written 3 ways: callbacks, `ContinueWith`, `async/await`. The `.html` files are its downloaded output |
| [Labs/Lab_05](Labs/Lab_05) | Polynomial multiplication: O(n^2) and Karatsuba, sequential and threaded |
| [Labs/Lab_06](Labs/Lab_06) | Hamiltonian cycle search in Java: sequential, manual threads, and ForkJoin |
| [Labs/Lab_07](Labs/Lab_07) | Polynomial and big-number multiplication with MPI (O(n^2) and Karatsuba) |
| [Labs/Lab_08](Labs/Lab_08) | Distributed shared memory: processes over TCP sockets with subscriptions, writes and compare-and-swap |
| [Labs/Bonus](Labs/Bonus) | Polynomial multiplication on the GPU with Apple Metal, with a LaTeX write-up |
| [Labs/Team_Project](Labs/Team_Project) | Graph k-coloring with backtracking: sequential, threads and a thread pool, with a LaTeX write-up |

## How to run

C and C++ labs without a Makefile:

```sh
cc -O2 -pthread Labs/Lab_01/warehouses.c -o warehouses && ./warehouses
c++ -std=c++17 -O2 -pthread Labs/Lab_03/matrix.cpp -o matrix && ./matrix
```

MPI (Lab_07), with Open MPI or MPICH installed:

```sh
mpic++ -std=c++17 -O2 Labs/Lab_07/main.cpp -o poly
mpirun -np 4 ./poly
```

Labs with a Makefile (Lab_08, Bonus, Team_Project):

```sh
cd Labs/Team_Project && make run
```

Java (Lab_06): from `Labs/Lab_06`, run `javac src/*.java && java src.Main`.

## Notes

- Some compiled binaries are committed (macOS arm64). Rebuild them on your machine.
- Lab_04 has only `main.cs`, no project file. Put it in a `dotnet new console` project to run it.
- The Bonus lab needs macOS (Metal framework).
- Lab_08 forks local processes that listen on ports 8000 and up.
- Parameters (sizes, thread counts, strategy) are mostly constants at the top of each source file.
