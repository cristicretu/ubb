#include <algorithm>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <random>
#include <thread>
#include <vector>

using namespace std;
typedef vector<int> Polynomial;

Polynomial generateRandomPolynomial(int size) {
  random_device rd;
  mt19937 gen(rd());
  uniform_int_distribution<> distributie(-10, 10);
  Polynomial poly(size);
  for (int i = 0; i < size; i++)
    poly[i] = 1;
  return poly;
}

void trimPolynomial(Polynomial &poly) {
  while (poly.size() > 1 && poly.back() == 0)
    poly.pop_back();
}

bool polynomialsEqual(const Polynomial &p1, const Polynomial &p2) {
  if (p1.size() != p2.size())
    return false;
  for (size_t i = 0; i < p1.size(); i++)
    if (p1[i] != p2[i])
      return false;
  return true;
}

Polynomial multiplyRegularSequential(const Polynomial &p1,
                                     const Polynomial &p2) {
  int n = p1.size(), m = p2.size();
  Polynomial result(n + m - 1, 0);
  for (int i = 0; i < n; i++)
    for (int j = 0; j < m; j++)
      result[i + j] += p1[i] * p2[j];
  return result;
}

Polynomial multiplyRegularParallel(const Polynomial &p1, const Polynomial &p2,
                                   int numThreads = 4) {
  int n = p1.size(), m = p2.size(), resultSize = n + m - 1;
  Polynomial result(resultSize, 0);

  auto workerFunction = [&](int start, int end) {
    for (int k = start; k < end; k++) {
      int startI = max(0, k - m + 1);
      int endI = min(k + 1, n);
      for (int i = startI; i < endI; i++) {
        int j = k - i;
        result[k] += p1[i] * p2[j];
      }
    }
  };

  vector<thread> threads;
  int chunkSize = (resultSize + numThreads - 1) / numThreads;
  for (int t = 0; t < numThreads; t++) {
    int start = t * chunkSize;
    int end = min(start + chunkSize, resultSize);
    if (start < resultSize)
      threads.emplace_back(workerFunction, start, end);
  }
  for (auto &thread : threads)
    thread.join();
  return result;
}

Polynomial addPolynomials(const Polynomial &p1, const Polynomial &p2) {
  int maxSize = max(p1.size(), p2.size());
  Polynomial result(maxSize, 0);
  for (size_t i = 0; i < p1.size(); i++)
    result[i] += p1[i];
  for (size_t i = 0; i < p2.size(); i++)
    result[i] += p2[i];
  return result;
}

Polynomial subtractPolynomials(const Polynomial &p1, const Polynomial &p2) {
  int maxSize = max(p1.size(), p2.size());
  Polynomial result(maxSize, 0);
  for (size_t i = 0; i < p1.size(); i++)
    result[i] += p1[i];
  for (size_t i = 0; i < p2.size(); i++)
    result[i] -= p2[i];
  return result;
}

Polynomial multiplyKaratsubaSequential(const Polynomial &p1,
                                       const Polynomial &p2) {
  int n = p1.size(), m = p2.size();
  if (n <= 10 || m <= 10)
    return multiplyRegularSequential(p1, p2);

  int mid = max(n, m) / 2;
  Polynomial low1(p1.begin(), p1.begin() + min((int)p1.size(), mid));
  Polynomial high1;
  if ((int)p1.size() > mid)
    high1 = Polynomial(p1.begin() + mid, p1.end());

  Polynomial low2(p2.begin(), p2.begin() + min((int)p2.size(), mid));
  Polynomial high2;
  if ((int)p2.size() > mid)
    high2 = Polynomial(p2.begin() + mid, p2.end());

  Polynomial z0 = multiplyKaratsubaSequential(low1, low2);
  Polynomial z2;
  if (!high1.empty() && !high2.empty())
    z2 = multiplyKaratsubaSequential(high1, high2);

  Polynomial sum1 = addPolynomials(low1, high1);
  Polynomial sum2 = addPolynomials(low2, high2);
  Polynomial z1 = multiplyKaratsubaSequential(sum1, sum2);
  z1 = subtractPolynomials(z1, z0);
  z1 = subtractPolynomials(z1, z2);

  int resultSize = n + m - 1;
  Polynomial result(resultSize, 0);
  for (size_t i = 0; i < z0.size(); i++)
    result[i] += z0[i];
  for (size_t i = 0; i < z1.size(); i++)
    if (i + mid < result.size())
      result[i + mid] += z1[i];
  for (size_t i = 0; i < z2.size(); i++)
    if (i + 2 * mid < result.size())
      result[i + 2 * mid] += z2[i];
  return result;
}

Polynomial multiplyKaratsubaParallelHelper(const Polynomial &p1,
                                           const Polynomial &p2, int depth,
                                           int maxDepth) {
  int n = p1.size(), m = p2.size();
  if (n <= 10 || m <= 10)
    return multiplyRegularSequential(p1, p2);

  int mid = max(n, m) / 2;
  Polynomial low1(p1.begin(), p1.begin() + min((int)p1.size(), mid));
  Polynomial high1;
  if ((int)p1.size() > mid)
    high1 = Polynomial(p1.begin() + mid, p1.end());

  Polynomial low2(p2.begin(), p2.begin() + min((int)p2.size(), mid));
  Polynomial high2;
  if ((int)p2.size() > mid)
    high2 = Polynomial(p2.begin() + mid, p2.end());

  if (depth < maxDepth) {
    Polynomial z0, z1, z2;
    Polynomial sum1 = addPolynomials(low1, high1);
    Polynomial sum2 = addPolynomials(low2, high2);

    thread t0([&]() {
      z0 = multiplyKaratsubaParallelHelper(low1, low2, depth + 1, maxDepth);
    });
    thread t1([&]() {
      z1 = multiplyKaratsubaParallelHelper(sum1, sum2, depth + 1, maxDepth);
    });
    thread t2([&]() {
      if (!high1.empty() && !high2.empty())
        z2 = multiplyKaratsubaParallelHelper(high1, high2, depth + 1, maxDepth);
    });

    t0.join();
    t1.join();
    t2.join();

    z1 = subtractPolynomials(z1, z0);
    z1 = subtractPolynomials(z1, z2);

    int resultSize = n + m - 1;
    Polynomial result(resultSize, 0);
    for (size_t i = 0; i < z0.size(); i++)
      result[i] += z0[i];
    for (size_t i = 0; i < z1.size(); i++)
      if (i + mid < result.size())
        result[i + mid] += z1[i];
    for (size_t i = 0; i < z2.size(); i++)
      if (i + 2 * mid < result.size())
        result[i + 2 * mid] += z2[i];
    return result;
  } else {
    return multiplyKaratsubaSequential(p1, p2);
  }
}

Polynomial multiplyKaratsubaParallel(const Polynomial &p1,
                                     const Polynomial &p2) {
  return multiplyKaratsubaParallelHelper(p1, p2, 0, 2);
}

int main() {
  int size;
  cout << "Polynomial size: ";
  cin >> size;

  Polynomial p1 = generateRandomPolynomial(size);
  Polynomial p2 = generateRandomPolynomial(size);
  Polynomial result1, result2, result3, result4;

  auto t1 = chrono::high_resolution_clock::now();
  result1 = multiplyRegularSequential(p1, p2);
  auto t2 = chrono::high_resolution_clock::now();
  result2 = multiplyRegularParallel(p1, p2, 4);
  auto t3 = chrono::high_resolution_clock::now();
  result3 = multiplyKaratsubaSequential(p1, p2);
  auto t4 = chrono::high_resolution_clock::now();
  result4 = multiplyKaratsubaParallel(p1, p2);
  auto t5 = chrono::high_resolution_clock::now();

  double time1 = chrono::duration<double, milli>(t2 - t1).count();
  double time2 = chrono::duration<double, milli>(t3 - t2).count();
  double time3 = chrono::duration<double, milli>(t4 - t3).count();
  double time4 = chrono::duration<double, milli>(t5 - t4).count();

  trimPolynomial(result1);
  trimPolynomial(result2);
  trimPolynomial(result3);
  trimPolynomial(result4);

  bool correct = polynomialsEqual(result1, result2) &&
                 polynomialsEqual(result2, result3) &&
                 polynomialsEqual(result3, result4);

  for (auto x : result4) {
    cout << x << ' ';
  }

  cout << "\n" << (correct ? "bine boss" : "nu e bine") << "\n\n";
  cout << left << setw(25) << "normal" << right << setw(12) << fixed
       << setprecision(2) << time1 << setw(12) << "1.00x\n";
  cout << left << setw(25) << "parallel" << right << setw(12) << time2
       << setw(12) << (time1 / time2) << "x\n";
  cout << left << setw(25) << "karatsuba sequential" << right << setw(12)
       << time3 << setw(12) << (time1 / time3) << "x\n";
  cout << left << setw(25) << "karatsuba parallel" << right << setw(12) << time4
       << setw(12) << (time1 / time4) << "x\n";

  return 0;
}
