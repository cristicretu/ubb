#include <iostream>
#include <mpi.h>
#include <vector>
using namespace std;

typedef vector<int> Poly;
typedef vector<int> BigNum;

const int BASE = 10;

BigNum normalize(BigNum n) {
  int carry = 0;
  for (size_t i = 0; i < n.size(); i++) {
    n[i] += carry;
    carry = n[i] / BASE;
    n[i] %= BASE;
    if (n[i] < 0) {
      n[i] += BASE;
      carry--;
    }
  }
  while (carry > 0) {
    n.push_back(carry % BASE);
    carry /= BASE;
  }
  while (n.size() > 1 && n.back() == 0)
    n.pop_back();
  return n;
}

Poly add(const Poly &a, const Poly &b) {
  Poly r(max(a.size(), b.size()), 0);
  for (size_t i = 0; i < a.size(); i++)
    r[i] += a[i];
  for (size_t i = 0; i < b.size(); i++)
    r[i] += b[i];
  return r;
}

Poly sub(const Poly &a, const Poly &b) {
  Poly r(max(a.size(), b.size()), 0);
  for (size_t i = 0; i < a.size(); i++)
    r[i] += a[i];
  for (size_t i = 0; i < b.size(); i++)
    r[i] -= b[i];
  return r;
}

Poly mul_seq(const Poly &a, const Poly &b) {
  int n = a.size(), m = b.size();
  Poly r(n + m - 1, 0);
  for (int i = 0; i < n; i++)
    for (int j = 0; j < m; j++)
      r[i + j] += a[i] * b[j];
  return r;
}

Poly karatsuba(const Poly &a, const Poly &b) {
  int n = a.size(), m = b.size();
  if (n < 64 || m < 64)
    return mul_seq(a, b);

  int mid = max(n, m) / 2;
  Poly lo1(a.begin(), a.begin() + min(n, mid));
  Poly hi1 = n > mid ? Poly(a.begin() + mid, a.end()) : Poly();
  Poly lo2(b.begin(), b.begin() + min(m, mid));
  Poly hi2 = m > mid ? Poly(b.begin() + mid, b.end()) : Poly();

  Poly z0 = karatsuba(lo1, lo2);
  Poly z2 = (!hi1.empty() && !hi2.empty()) ? karatsuba(hi1, hi2) : Poly();
  Poly z1 = sub(sub(karatsuba(add(lo1, hi1), add(lo2, hi2)), z0), z2);

  Poly r(n + m - 1, 0);
  for (size_t i = 0; i < z0.size(); i++)
    r[i] += z0[i];
  for (size_t i = 0; i < z1.size(); i++)
    if (i + mid < r.size())
      r[i + mid] += z1[i];
  for (size_t i = 0; i < z2.size(); i++)
    if (i + 2 * mid < r.size())
      r[i + 2 * mid] += z2[i];
  return r;
}

void send_poly(const Poly &p, int dest, int tag) {
  int sz = p.size();
  MPI_Send(&sz, 1, MPI_INT, dest, tag, MPI_COMM_WORLD);
  MPI_Send(p.data(), sz, MPI_INT, dest, tag + 1, MPI_COMM_WORLD);
}

Poly recv_poly(int src, int tag) {
  int sz;
  MPI_Recv(&sz, 1, MPI_INT, src, tag, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
  Poly p(sz);
  MPI_Recv(p.data(), sz, MPI_INT, src, tag + 1, MPI_COMM_WORLD,
           MPI_STATUS_IGNORE);
  return p;
}

Poly mul_mpi(const Poly &a, const Poly &b, int rank, int nprocs) {
  int n = a.size(), m = b.size(), rsize = n + m - 1;
  int chunk = (rsize + nprocs - 1) / nprocs;

  if (rank == 0) {
    for (int p = 1; p < nprocs; p++) {
      send_poly(a, p, 0);
      send_poly(b, p, 10);
    }

    Poly result(rsize, 0);
    for (int k = 0; k < min(chunk, rsize); k++) {
      for (int i = max(0, k - m + 1); i < min(k + 1, n); i++)
        result[k] += a[i] * b[k - i];
    }

    for (int p = 1; p < nprocs; p++) {
      int ps = p * chunk, pe = min(ps + chunk, rsize);
      if (ps < rsize) {
        vector<int> partial(pe - ps);
        MPI_Recv(partial.data(), pe - ps, MPI_INT, p, 100, MPI_COMM_WORLD,
                 MPI_STATUS_IGNORE);
        for (int i = 0; i < pe - ps; i++)
          result[ps + i] = partial[i];
      }
    }
    return result;
  }

  Poly pa = recv_poly(0, 0), pb = recv_poly(0, 10);
  int pn = pa.size(), pm = pb.size(), prsize = pn + pm - 1;
  int start = rank * chunk, end = min(start + chunk, prsize);

  vector<int> partial(end - start, 0);
  for (int k = start; k < end; k++) {
    for (int i = max(0, k - pm + 1); i < min(k + 1, pn); i++)
      partial[k - start] += pa[i] * pb[k - i];
  }
  MPI_Send(partial.data(), end - start, MPI_INT, 0, 100, MPI_COMM_WORLD);
  return {};
}

Poly karatsuba_mpi(const Poly &a, const Poly &b, int rank, int nprocs) {
  int n = a.size(), m = b.size();

  if (rank == 0) {
    for (int p = 1; p < nprocs; p++) {
      send_poly(a, p, 0);
      send_poly(b, p, 10);
    }

    int mid = max(n, m) / 2;
    Poly lo1(a.begin(), a.begin() + min(n, mid));
    Poly hi1 = n > mid ? Poly(a.begin() + mid, a.end()) : Poly();
    Poly lo2(b.begin(), b.begin() + min(m, mid));
    Poly hi2 = m > mid ? Poly(b.begin() + mid, b.end()) : Poly();
    Poly sum1 = add(lo1, hi1), sum2 = add(lo2, hi2);

    int workers = min(3, nprocs - 1);
    if (workers >= 1) {
      send_poly(lo1, 1, 20);
      send_poly(lo2, 1, 30);
    }
    if (workers >= 2) {
      send_poly(sum1, 2, 20);
      send_poly(sum2, 2, 30);
    }
    if (workers >= 3) {
      send_poly(hi1, 3, 20);
      send_poly(hi2, 3, 30);
    }

    Poly z0 = workers >= 1 ? recv_poly(1, 40) : karatsuba(lo1, lo2);
    Poly z1 = workers >= 2 ? recv_poly(2, 40) : karatsuba(sum1, sum2);
    Poly z2 = workers >= 3
                  ? recv_poly(3, 40)
                  : (hi1.empty() || hi2.empty() ? Poly() : karatsuba(hi1, hi2));

    z1 = sub(sub(z1, z0), z2);
    Poly r(n + m - 1, 0);
    for (size_t i = 0; i < z0.size(); i++)
      r[i] += z0[i];
    for (size_t i = 0; i < z1.size(); i++)
      if (i + mid < r.size())
        r[i + mid] += z1[i];
    for (size_t i = 0; i < z2.size(); i++)
      if (i + 2 * mid < r.size())
        r[i + 2 * mid] += z2[i];
    return r;
  }

  recv_poly(0, 0);
  recv_poly(0, 10);
  if (rank <= 3) {
    Poly pa = recv_poly(0, 20), pb = recv_poly(0, 30);
    send_poly(karatsuba(pa, pb), 0, 40);
  }
  return {};
}

BigNum bignum_mul_seq(const BigNum &a, const BigNum &b) {
  return normalize(mul_seq(a, b));
}
BigNum bignum_karatsuba(const BigNum &a, const BigNum &b) {
  return normalize(karatsuba(a, b));
}
BigNum bignum_mul_mpi(const BigNum &a, const BigNum &b, int rank, int nprocs) {
  return normalize(mul_mpi(a, b, rank, nprocs));
}
BigNum bignum_karatsuba_mpi(const BigNum &a, const BigNum &b, int rank,
                            int nprocs) {
  return normalize(karatsuba_mpi(a, b, rank, nprocs));
}

int main(int argc, char **argv) {
  MPI_Init(&argc, &argv);
  int rank, nprocs;
  MPI_Comm_rank(MPI_COMM_WORLD, &rank);
  MPI_Comm_size(MPI_COMM_WORLD, &nprocs);

  int sz = argc > 1 ? atoi(argv[1]) : 10000;
  Poly p1(sz, 1), p2(sz, 1);

  double t0 = MPI_Wtime();
  Poly r1 = rank == 0 ? mul_seq(p1, p2) : Poly();
  double t1 = MPI_Wtime();
  Poly r2 = mul_mpi(p1, p2, rank, nprocs);
  double t2 = MPI_Wtime();
  Poly r3 = rank == 0 ? karatsuba(p1, p2) : Poly();
  double t3 = MPI_Wtime();
  Poly r4 = karatsuba_mpi(p1, p2, rank, nprocs);
  double t4 = MPI_Wtime();

  if (rank == 0) {
    bool ok = r1 == r2 && r2 == r3 && r3 == r4;
    printf("POLYNOMIAL (n=%d, procs=%d) %s n", sz, nprocs,
           ok ? "bun" : "varza");
    printf("seq O(n²):     %.2fms\n", (t1 - t0) * 1000);
    printf("mpi O(n²):     %.2fms (%.1fx)\n", (t2 - t1) * 1000,
           (t1 - t0) / (t2 - t1));
    printf("karatsuba:     %.2fms (%.1fx)\n", (t3 - t2) * 1000,
           (t1 - t0) / (t3 - t2));
    printf("karatsuba mpi: %.2fms (%.1fx)\n", (t4 - t3) * 1000,
           (t1 - t0) / (t4 - t3));
  }

  int digits = sz / 2;
  BigNum a(digits, 9), b(digits, 9);

  double t5 = MPI_Wtime();
  BigNum br1 = rank == 0 ? bignum_mul_seq(a, b) : BigNum();
  double t6 = MPI_Wtime();
  BigNum br2 = bignum_mul_mpi(a, b, rank, nprocs);
  double t7 = MPI_Wtime();
  BigNum br3 = rank == 0 ? bignum_karatsuba(a, b) : BigNum();
  double t8 = MPI_Wtime();
  BigNum br4 = bignum_karatsuba_mpi(a, b, rank, nprocs);
  double t9 = MPI_Wtime();

  if (rank == 0) {
    bool ok = br1 == br2 && br2 == br3 && br3 == br4;
    printf("\n BIGNUM (%d digits, procs=%d) %sn", digits, nprocs,
           ok ? "bine boss" : "nu e bine");
    printf("seq O(n²):     %.2fms\n", (t6 - t5) * 1000);
    printf("mpi O(n²):     %.2fms (%.1fx)\n", (t7 - t6) * 1000,
           (t6 - t5) / (t7 - t6));
    printf("karatsuba:     %.2fms (%.1fx)\n", (t8 - t7) * 1000,
           (t6 - t5) / (t8 - t7));
    printf("karatsuba mpi: %.2fms (%.1fx)\n", (t9 - t8) * 1000,
           (t6 - t5) / (t9 - t8));
  }

  MPI_Finalize();

  // if (rank == 0) {
  //   cout << "r1: ";
  //   for (int i = 0; i < r1.size(); i++)
  //     cout << r1[i] << " ";
  //   cout << endl;
  //   cout << "r2: ";
  //   for (int i = 0; i < r2.size(); i++)
  //     cout << r2[i] << " ";
  //   cout << endl;
  //   cout << "r3: ";
  //   for (int i = 0; i < r3.size(); i++)
  //     cout << r3[i] << " ";
  //   cout << endl;
  //   cout << "r4: ";
  //   for (int i = 0; i < r4.size(); i++)
  //     cout << r4[i] << " ";
  //   cout << endl;
  // }
  return 0;
}
