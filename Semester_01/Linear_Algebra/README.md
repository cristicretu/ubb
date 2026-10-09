# Linear Algebra

Semester 1 · Year 1 · Python (NumPy), C++

Algebraic structures (groups, rings, fields), vector spaces, bases, linear maps, matrices and systems. The labs are small counting programs over sets and over Z2^n, each run on five input files.

## Contents

| Folder | What it is |
| --- | --- |
| [Labs/Lab_01](Labs/Lab_01) | Partitions of a set {a1..an} and their equivalence relations |
| [Labs/Lab_02](Labs/Lab_02) | Count associative operations on a set of size n, print their tables for n <= 4 (C++, multithreaded). `M/` is an alternative version |
| [Labs/Lab_03](Labs/Lab_03) | Number of bases of Z2^n over Z2 and the vectors of each basis. `A/` and `M/` are alternative versions; `Untitled.ipynb` is scratch work |
| [Labs/Lab_04](Labs/Lab_04) | Number of k-dimensional subspaces of Z2^n (Gaussian binomial) and a basis for each |
| [Labs/Lab_05](Labs/Lab_05) | Number of distinct reduced row echelon form matrices in Z2^(m x n) |
| [Exercices.pdf](Exercices.pdf) | Seminar problem sheets |
| [Seminars.pdf](Seminars.pdf) | Seminar notes with solutions (48 pages) |
| [Exam](Exam) | Photos of past written exam subjects |

## How to run

Each lab reads `n` (or `m n`) from `0X_input.txt` and writes the result to the matching `0X_output.txt` (the committed outputs are already there).

```sh
cd Labs/Lab_03
pip install numpy
python3 main.py 01_input.txt
```

Lab_02 is C++:

```sh
cd Labs/Lab_02
g++ -std=c++17 -O2 -pthread main.cpp -o main
./main 01_input.txt
```

## Notes

- The `main` binaries in Lab_02 are compiled for macOS arm64; rebuild them on other systems.
- Lab_02 brute-forces every n x n operation table (n^(n^2) of them), so large inputs are slow.
