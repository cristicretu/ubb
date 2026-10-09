# Graph Algorithms

Semester 2 · Year 1 · C++, Python

Directed graphs and the classic algorithms on them: traversals, connected components, shortest paths, DAGs, Hamiltonian cycles. All labs grow the same `Graph` class (inbound/outbound adjacency lists plus edge costs) with a console menu.

## Contents

| Folder | What it is |
| --- | --- |
| [Labs/Lab_01](Labs/Lab_01) | The `Graph` class: add/remove vertices and edges, degrees, inbound/outbound edges, costs, copy, read/write file, random graphs. Report in `main.tex`/`main.pdf`; `cretu_cristian/` and the `.zip` are the submitted copy |
| [Labs/Lab_02](Labs/Lab_02) | Adds backwards BFS and strongly connected components (DFS on outbound and inbound edges). `manual exec.pdf` is a hand run |
| [Labs/Lab_03](Labs/Lab_03) | Adds lowest cost walk (matrix multiplication), Dijkstra, negative cycle detection, counting walks of minimum cost |
| [Labs/Lab_04](Labs/Lab_04) | Adds DAG check, highest cost path, number of distinct paths and distinct lowest cost paths |
| [Labs/Lab_05](Labs/Lab_05) | Adds the lowest cost Hamiltonian cycle (branch and bound), tested on `hamilton.in` / `nhamilton.in` |
| [Labs/Practice](Labs/Practice) | Small adjacency-matrix graph read from `graph.txt` |
| [Exam](Exam) | Photos of past exam subjects and a cheat sheet |

## How to run

The C++ version is the complete one; `main.py` in each lab only has the Lab_01 operations.

```sh
cd Labs/Lab_05
g++ -std=c++17 -O2 main.cpp -o graph && ./graph
```

From the menu, read a graph file such as `example.txt` or `graph1k.txt`. For the Python version: `python3 main.py`.

## Notes

- Graph file format: first line `n m`, then `m` lines `source target cost`.
- In Lab_04 and Lab_05 the menu entries 17-25 are commented out, but the functions are still in the code.
- `graph100k.txt` (Lab_02, Lab_03) has 100k vertices and 400k edges, useful for timing.
