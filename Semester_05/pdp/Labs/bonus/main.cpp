#define CL_SILENCE_DEPRECATION
#include <OpenCL/opencl.h>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <vector>

using namespace std;
typedef vector<int> Poly;

const char *kernelSrc = R"(
kernel void poly_mul(global const int* p1, global const int* p2, global int* result, int n, int m) {
    int k = get_global_id(0);
    int result_size = n + m - 1;
    if (k >= result_size) return;
    
    int sum = 0;
    int start = (k >= m - 1) ? (k - m + 1) : 0;
    int end = (k + 1 < n) ? (k + 1) : n;
    
    for (int i = start; i < end; i++) {
        sum += p1[i] * p2[k - i];
    }
    result[k] = sum;
}
)";

cl_context ctx;
cl_command_queue cmdq;
cl_program program;
cl_kernel kernel;

void initCL() {
  cl_platform_id platform;
  cl_device_id device;
  clGetPlatformIDs(1, &platform, NULL);
  clGetDeviceIDs(platform, CL_DEVICE_TYPE_GPU, 1, &device, NULL);

  char name[128];
  clGetDeviceInfo(device, CL_DEVICE_NAME, 128, name, NULL);
  cout << "OpenCL GPU: " << name << "\n\n";

  ctx = clCreateContext(NULL, 1, &device, NULL, NULL, NULL);
  cmdq = clCreateCommandQueue(ctx, device, 0, NULL);
  program = clCreateProgramWithSource(ctx, 1, &kernelSrc, NULL, NULL);
  clBuildProgram(program, 1, &device, NULL, NULL, NULL);
  kernel = clCreateKernel(program, "poly_mul", NULL);
}

Poly mulGPU(const Poly &p1, const Poly &p2) {
  int n = p1.size(), m = p2.size();
  int rsize = n + m - 1;

  cl_mem b1 = clCreateBuffer(ctx, CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR,
                             n * sizeof(int), (void *)p1.data(), NULL);
  cl_mem b2 = clCreateBuffer(ctx, CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR,
                             m * sizeof(int), (void *)p2.data(), NULL);
  cl_mem br =
      clCreateBuffer(ctx, CL_MEM_WRITE_ONLY, rsize * sizeof(int), NULL, NULL);

  clSetKernelArg(kernel, 0, sizeof(cl_mem), &b1);
  clSetKernelArg(kernel, 1, sizeof(cl_mem), &b2);
  clSetKernelArg(kernel, 2, sizeof(cl_mem), &br);
  clSetKernelArg(kernel, 3, sizeof(int), &n);
  clSetKernelArg(kernel, 4, sizeof(int), &m);

  size_t global = rsize;
  clEnqueueNDRangeKernel(cmdq, kernel, 1, NULL, &global, NULL, 0, NULL, NULL);
  clFinish(cmdq);

  Poly result(rsize);
  clEnqueueReadBuffer(cmdq, br, CL_TRUE, 0, rsize * sizeof(int), result.data(),
                      0, NULL, NULL);

  clReleaseMemObject(b1);
  clReleaseMemObject(b2);
  clReleaseMemObject(br);
  return result;
}

Poly mulCPU(const Poly &p1, const Poly &p2) {
  int n = p1.size(), m = p2.size();
  Poly r(n + m - 1, 0);
  for (int i = 0; i < n; i++)
    for (int j = 0; j < m; j++)
      r[i + j] += p1[i] * p2[j];
  return r;
}

Poly karatsubaCPU(const Poly &p1, const Poly &p2) {
  int n = p1.size(), m = p2.size();
  if (n <= 64 || m <= 64)
    return mulCPU(p1, p2);

  int mid = max(n, m) / 2;
  Poly lo1(p1.begin(), p1.begin() + min(n, mid));
  Poly hi1 = (n > mid) ? Poly(p1.begin() + mid, p1.end()) : Poly{0};
  Poly lo2(p2.begin(), p2.begin() + min(m, mid));
  Poly hi2 = (m > mid) ? Poly(p2.begin() + mid, p2.end()) : Poly{0};

  Poly z0 = karatsubaCPU(lo1, lo2);
  Poly z2 = karatsubaCPU(hi1, hi2);

  Poly s1(max(lo1.size(), hi1.size()), 0), s2(max(lo2.size(), hi2.size()), 0);
  for (size_t i = 0; i < lo1.size(); i++)
    s1[i] += lo1[i];
  for (size_t i = 0; i < hi1.size(); i++)
    s1[i] += hi1[i];
  for (size_t i = 0; i < lo2.size(); i++)
    s2[i] += lo2[i];
  for (size_t i = 0; i < hi2.size(); i++)
    s2[i] += hi2[i];

  Poly z1 = karatsubaCPU(s1, s2);
  for (size_t i = 0; i < z0.size() && i < z1.size(); i++)
    z1[i] -= z0[i];
  for (size_t i = 0; i < z2.size() && i < z1.size(); i++)
    z1[i] -= z2[i];

  Poly r(n + m - 1, 0);
  for (size_t i = 0; i < z0.size(); i++)
    r[i] += z0[i];
  for (size_t i = 0; i < z1.size() && i + mid < r.size(); i++)
    r[i + mid] += z1[i];
  for (size_t i = 0; i < z2.size() && i + 2 * mid < r.size(); i++)
    r[i + 2 * mid] += z2[i];
  return r;
}

bool eq(const Poly &a, const Poly &b) {
  if (a.size() != b.size())
    return false;
  for (size_t i = 0; i < a.size(); i++)
    if (a[i] != b[i])
      return false;
  return true;
}

int main() {
  srand(42);
  initCL();

  vector<int> sizes = {1000, 5000, 10000, 20000, 50000};

  cout << left << setw(10) << "n" << right << setw(14) << "CPU O(n²)"
       << setw(14) << "CPU Karat" << setw(14) << "GPU O(n²)" << setw(10)
       << "speedup\n";
  cout << string(62, '-') << "\n";

  for (int sz : sizes) {
    Poly p1(sz), p2(sz);
    for (int i = 0; i < sz; i++)
      p1[i] = rand() % 21 - 10, p2[i] = rand() % 21 - 10;

    auto t0 = chrono::high_resolution_clock::now();
    Poly r1 = mulCPU(p1, p2);
    auto t1 = chrono::high_resolution_clock::now();
    Poly r2 = karatsubaCPU(p1, p2);
    auto t2 = chrono::high_resolution_clock::now();
    Poly r3 = mulGPU(p1, p2);
    auto t3 = chrono::high_resolution_clock::now();

    double cpu = chrono::duration<double, milli>(t1 - t0).count();
    double karat = chrono::duration<double, milli>(t2 - t1).count();
    double gpu = chrono::duration<double, milli>(t3 - t2).count();

    cout << left << setw(10) << sz << right << fixed << setprecision(2)
         << setw(12) << cpu << "ms" << setw(12) << karat << "ms" << setw(12)
         << gpu << "ms" << setw(8) << (cpu / gpu) << "x"
         << (eq(r1, r3) ? " ✓" : " ✗") << "\n";
  }
}



