# Polynomial Multiplication - OpenCL

## Build & Run
```bash
make && ./poly_gpu
```

## Algorithms

### Regular O(n²) - GPU
Each thread computes one output coefficient:
```
result[k] = Σ p1[i] * p2[k-i]  for valid i
```
No sync needed - each thread writes to unique index.

### Karatsuba O(n^1.585) - CPU
Divide and conquer:
```
z0 = low1 * low2
z2 = high1 * high2
z1 = (low1+high1)(low2+high2) - z0 - z2
result = z0 + z1*x^mid + z2*x^(2*mid)
```

## Sync
- GPU kernel: none (each thread writes unique index)
- OpenCL: `clFinish()` waits for kernel completion

## Results (M4 Max)
```
n             CPU O(n²)     CPU Karat    GPU O(n²)  speedup
--------------------------------------------------------------
1000              0.06ms        0.06ms       10.36ms    0.01x ✓
5000              1.48ms        0.76ms        2.39ms    0.62x ✓
10000             5.74ms        2.15ms        3.25ms    1.77x ✓
20000            42.96ms        6.93ms        3.06ms   14.04x ✓
50000           261.72ms       33.98ms       24.73ms   10.58x ✓
```

GPU wins at n > 7000. First call has high overhead (JIT compile). Subsequent calls faster.



