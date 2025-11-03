#include <chrono>
#include <iomanip>
#include <iostream>
#include <thread>
#include <vector>
using namespace std;

using Matrix = vector<vector<double>>;

const bool DEBUG_MODE = false;
const int ROW_SPLIT = 0;
const int COL_SPLIT = 1;
const int KTH_SPLIT = 2;

const int NUM_SIZE = 1000;
const int NUM_THREADS = 12;
const int STRATEGY = COL_SPLIT;

double compute_element(const Matrix &A, const Matrix &B, int row, int col) {
  double result = 0;
  int common_dim = A[0].size();
  for (int k = 0; k < common_dim; ++k)
    result += A[row][k] * B[k][col];
  return result;
}

tuple<int, int, int> get_chunk_indices(int tid, int nthreads,
                                       int total_elements) {
  int chunk_size = total_elements / nthreads;
  int start_idx = tid * chunk_size;
  int end_idx = (tid == nthreads - 1) ? total_elements : start_idx + chunk_size;

  return {start_idx, end_idx, chunk_size};
}

void compute_row_split(const Matrix &A, const Matrix &B, Matrix &C, int tid,
                       int nthreads) {
  auto [start_idx, end_idx, chunk_size] =
      get_chunk_indices(tid, nthreads, C.size() * C[0].size());
  int rows = C.size();
  int cols = C[0].size();

  for (int idx = start_idx; idx < end_idx; ++idx) {
    int r = idx / cols;
    int c = idx % cols;
    C[r][c] = compute_element(A, B, r, c);
    if (DEBUG_MODE)
      cout << "Thread " << tid << " computed C[" << r << "][" << c << "]\n";
  }
}

void compute_col_split(const Matrix &A, const Matrix &B, Matrix &C, int tid,
                       int nthreads) {
  auto [start_idx, end_idx, chunk_size] =
      get_chunk_indices(tid, nthreads, C.size() * C[0].size());
  int rows = C.size();
  int cols = C[0].size();

  for (int idx = start_idx; idx < end_idx; ++idx) {
    int c = idx / rows;
    int r = idx % rows;
    C[r][c] = compute_element(A, B, r, c);
    if (DEBUG_MODE)
      cout << "Thread " << tid << " computed C[" << r << "][" << c << "]\n";
  }
}

void compute_kth_split(const Matrix &A, const Matrix &B, Matrix &C, int tid,
                       int nthreads) {
  int rows = C.size();
  int cols = C[0].size();
  int total_elements = rows * cols;

  for (int idx = tid; idx < total_elements; idx += nthreads) {
    int r = idx / cols;
    int c = idx % cols;
    C[r][c] = compute_element(A, B, r, c);
    if (DEBUG_MODE)
      cout << "Thread " << tid << " computed C[" << r << "][" << c << "]\n";
  }
}

void worker(const Matrix &A, const Matrix &B, Matrix &C, int tid, int nthreads,
            int strategy) {
  if (strategy == ROW_SPLIT)
    compute_row_split(A, B, C, tid, nthreads);
  else if (strategy == COL_SPLIT)
    compute_col_split(A, B, C, tid, nthreads);
  else if (strategy == KTH_SPLIT)
    compute_kth_split(A, B, C, tid, nthreads);
}

bool verify_result(const Matrix &A, const Matrix &B, const Matrix &C) {
  int rows = C.size();
  int cols = C[0].size();
  for (int i = 0; i < rows; ++i) {
    for (int j = 0; j < cols; ++j) {
      double expected = compute_element(A, B, i, j);
      if (abs(C[i][j] - expected) > 1e-9)
        return false;
    }
  }
  return true;
}

void assignMatrix(Matrix &A, int rows, int cols) {
  for (int i = 0; i < rows; ++i)
    for (int j = 0; j < cols; ++j)
      A[i][j] = 1.5;
}

void printMatrix(const Matrix &A) {
  for (int i = 0; i < A.size(); ++i) {
    for (int j = 0; j < A[0].size(); ++j)
      cout << A[i][j] << " ";
    cout << "\n";
  }
}

void solve(int rows_a, int cols_a, int cols_b, int nthreads, int strategy) {
  Matrix A(rows_a, vector<double>(cols_a, 1.0));
  Matrix B(cols_a, vector<double>(cols_b, 1.0));
  Matrix C(rows_a, vector<double>(cols_b, 1.0));

  auto start = chrono::high_resolution_clock::now();

  vector<thread> workers;
  for (int t = 0; t < nthreads; ++t)
    workers.emplace_back(worker, cref(A), cref(B), ref(C), t, nthreads,
                         strategy);

  for (auto &w : workers)
    w.join();

  auto end = chrono::high_resolution_clock::now();
  auto duration =
      chrono::duration_cast<chrono::microseconds>(end - start).count();

  cout << "Time: " << fixed << setprecision(2) << duration / 1000.0 << " ms\n";

  if (verify_result(A, B, C))
    cout << "BINEBOSSSSS\n";
  else
    cout << "NAH NAH NAH\n";
}

int main() {
  solve(NUM_SIZE, NUM_SIZE, NUM_SIZE, NUM_THREADS, STRATEGY);
  return 0;
}
