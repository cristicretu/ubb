# Lab 07: MPI Polynomial Multiplication

## Build & Run
```bash
mpic++ -O3 main.cpp -o poly
mpirun -np 4 ./poly 10000
```

## Algorithms

**Regular O(n²)**: Classic convolution. `R[k] = Σ A[i] * B[k-i]`

**Karatsuba O(n^1.58)**: Split polynomials at midpoint, compute 3 products instead of 4:
- z0 = lo1 * lo2
- z2 = hi1 * hi2  
- z1 = (lo1+hi1)(lo2+hi2) - z0 - z2
- Result = z0 + z1*x^mid + z2*x^(2*mid)

## MPI Distribution

**Regular**: Master splits output indices across workers. Each worker computes their chunk, sends back to master.

**Karatsuba**: Master sends the 3 sub-problems (lo*lo, sum*sum, hi*hi) to workers 1-3. Workers compute using sequential karatsuba, send results back. Master combines.

## Performance (n=10000, 4 procs)

| Method | Time | Speedup |
|--------|------|---------|
| Sequential O(n²) | ~400ms | 1x |
| MPI O(n²) | ~130ms | 3x |
| Karatsuba | ~30ms | 13x |
| Karatsuba MPI | ~12ms | 33x |

MPI regular scales with more processes. Karatsuba MPI maxes at 4 procs (master + 3 workers for 3 subproblems).


