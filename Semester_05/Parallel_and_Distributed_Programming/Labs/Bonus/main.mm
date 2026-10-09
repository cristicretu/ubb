#import <Foundation/Foundation.h>
#import <Metal/Metal.h>
#include <chrono>
#include <cstdio>
#include <random>
#include <vector>

using Poly = std::vector<int>;
using Clock = std::chrono::high_resolution_clock;

// 1 th per coeff
// 1 thread per multiplication

static const char *shader_source = R"(
#include <metal_stdlib>
using namespace metal;

kernel void poly_multiply_regular(
    device const int* p1 [[buffer(0)]],
    device const int* p2 [[buffer(1)]],
    device atomic_int* result [[buffer(2)]],
    constant int& n [[buffer(3)]],
    constant int& m [[buffer(4)]],
    uint tid [[thread_position_in_grid]]
) {
    int result_size = n + m - 1;
    if (tid >= (uint)result_size) return;

    int k = tid;
    int sum = 0;
    int start_i = max(0, k - m + 1);
    int end_i = min(k + 1, n);

    for (int i = start_i; i < end_i; i++) {
        int j = k - i;
        sum += p1[i] * p2[j];
    }

    atomic_fetch_add_explicit(&result[k], sum, memory_order_relaxed);
}

kernel void poly_multiply_tiled(
    device const int* p1 [[buffer(0)]],
    device const int* p2 [[buffer(1)]],
    device atomic_int* result [[buffer(2)]],
    constant int& n [[buffer(3)]],
    constant int& m [[buffer(4)]],
    uint2 tid [[thread_position_in_grid]]
) {
    int i = tid.x;
    int j = tid.y;

    if (i >= n || j >= m) return;

    atomic_fetch_add_explicit(&result[i + j], p1[i] * p2[j], memory_order_relaxed);
}
)";

Poly random_poly(int size, unsigned seed = 42) {
  Poly p(size);
  std::mt19937 rng(seed);
  std::uniform_int_distribution<int> dist(-10, 10);
  for (int i = 0; i < size; i++)
    p[i] = dist(rng);
  return p;
}

Poly ones_poly(int size) { return Poly(size, 1); }

Poly multiply_cpu(const Poly &p1, const Poly &p2) {
  int n = p1.size(), m = p2.size();
  Poly result(n + m - 1, 0);
  for (int i = 0; i < n; i++)
    for (int j = 0; j < m; j++)
      result[i + j] += p1[i] * p2[j];
  return result;
}

Poly karatsuba(const Poly &p1, const Poly &p2) {
  int n = p1.size(), m = p2.size();
  if (n <= 32 || m <= 32)
    return multiply_cpu(p1, p2);

  int mid = std::max(n, m) / 2;

  Poly low1(p1.begin(), p1.begin() + std::min((int)p1.size(), mid));
  Poly high1;
  if ((int)p1.size() > mid)
    high1 = Poly(p1.begin() + mid, p1.end());

  Poly low2(p2.begin(), p2.begin() + std::min((int)p2.size(), mid));
  Poly high2;
  if ((int)p2.size() > mid)
    high2 = Poly(p2.begin() + mid, p2.end());

  Poly z0 = karatsuba(low1, low2);
  Poly z2;
  if (!high1.empty() && !high2.empty())
    z2 = karatsuba(high1, high2);

  Poly sum1(std::max(low1.size(), high1.size()), 0);
  Poly sum2(std::max(low2.size(), high2.size()), 0);
  for (size_t i = 0; i < low1.size(); i++)
    sum1[i] += low1[i];
  for (size_t i = 0; i < high1.size(); i++)
    sum1[i] += high1[i];
  for (size_t i = 0; i < low2.size(); i++)
    sum2[i] += low2[i];
  for (size_t i = 0; i < high2.size(); i++)
    sum2[i] += high2[i];

  Poly z1 = karatsuba(sum1, sum2);
  for (size_t i = 0; i < z0.size() && i < z1.size(); i++)
    z1[i] -= z0[i];
  for (size_t i = 0; i < z2.size() && i < z1.size(); i++)
    z1[i] -= z2[i];

  Poly result(n + m - 1, 0);
  for (size_t i = 0; i < z0.size(); i++)
    result[i] += z0[i];
  for (size_t i = 0; i < z1.size() && i + mid < result.size(); i++)
    result[i + mid] += z1[i];
  for (size_t i = 0; i < z2.size() && i + 2 * mid < result.size(); i++)
    result[i + 2 * mid] += z2[i];

  return result;
}

class MetalCompute {
public:
  id<MTLDevice> device;
  id<MTLCommandQueue> queue;
  id<MTLComputePipelineState> regular_pipeline;
  id<MTLComputePipelineState> tiled_pipeline;

  MetalCompute() {
    device = MTLCreateSystemDefaultDevice();
    if (!device) {
      printf("Metal not available\n");
      exit(1);
    }
    queue = [device newCommandQueue];

    NSError *error = nil;
    NSString *source = [NSString stringWithUTF8String:shader_source];
    id<MTLLibrary> library = [device newLibraryWithSource:source
                                                  options:nil
                                                    error:&error];

    if (!library) {
      printf("Failed to compile shaders: %s\n",
             [[error localizedDescription] UTF8String]);
      exit(1);
    }

    id<MTLFunction> regular_fn =
        [library newFunctionWithName:@"poly_multiply_regular"];
    id<MTLFunction> tiled_fn =
        [library newFunctionWithName:@"poly_multiply_tiled"];

    regular_pipeline = [device newComputePipelineStateWithFunction:regular_fn
                                                             error:&error];
    tiled_pipeline = [device newComputePipelineStateWithFunction:tiled_fn
                                                           error:&error];
  }

  Poly multiply_regular(const Poly &p1, const Poly &p2) {
    int n = (int)p1.size();
    int m = (int)p2.size();
    int result_size = n + m - 1;

    id<MTLBuffer> buf_p1 =
        [device newBufferWithBytes:p1.data()
                            length:n * sizeof(int)
                           options:MTLResourceStorageModeShared];
    id<MTLBuffer> buf_p2 =
        [device newBufferWithBytes:p2.data()
                            length:m * sizeof(int)
                           options:MTLResourceStorageModeShared];
    id<MTLBuffer> buf_result =
        [device newBufferWithLength:result_size * sizeof(int)
                            options:MTLResourceStorageModeShared];
    memset([buf_result contents], 0, result_size * sizeof(int));

    id<MTLCommandBuffer> cmd = [queue commandBuffer];
    id<MTLComputeCommandEncoder> encoder = [cmd computeCommandEncoder];

    [encoder setComputePipelineState:regular_pipeline];
    [encoder setBuffer:buf_p1 offset:0 atIndex:0];
    [encoder setBuffer:buf_p2 offset:0 atIndex:1];
    [encoder setBuffer:buf_result offset:0 atIndex:2];
    [encoder setBytes:&n length:sizeof(int) atIndex:3];
    [encoder setBytes:&m length:sizeof(int) atIndex:4];

    NSUInteger threads_per_group =
        regular_pipeline.maxTotalThreadsPerThreadgroup;
    MTLSize grid = MTLSizeMake(result_size, 1, 1);
    MTLSize group =
        MTLSizeMake(std::min((NSUInteger)result_size, threads_per_group), 1, 1);

    [encoder dispatchThreads:grid threadsPerThreadgroup:group];
    [encoder endEncoding];
    [cmd commit];
    [cmd waitUntilCompleted];

    Poly result(result_size);
    memcpy(result.data(), [buf_result contents], result_size * sizeof(int));
    return result;
  }

  Poly multiply_tiled(const Poly &p1, const Poly &p2) {
    int n = (int)p1.size();
    int m = (int)p2.size();
    int result_size = n + m - 1;

    id<MTLBuffer> buf_p1 =
        [device newBufferWithBytes:p1.data()
                            length:n * sizeof(int)
                           options:MTLResourceStorageModeShared];
    id<MTLBuffer> buf_p2 =
        [device newBufferWithBytes:p2.data()
                            length:m * sizeof(int)
                           options:MTLResourceStorageModeShared];
    id<MTLBuffer> buf_result =
        [device newBufferWithLength:result_size * sizeof(int)
                            options:MTLResourceStorageModeShared];
    memset([buf_result contents], 0, result_size * sizeof(int));

    id<MTLCommandBuffer> cmd = [queue commandBuffer];
    id<MTLComputeCommandEncoder> encoder = [cmd computeCommandEncoder];

    [encoder setComputePipelineState:tiled_pipeline];
    [encoder setBuffer:buf_p1 offset:0 atIndex:0];
    [encoder setBuffer:buf_p2 offset:0 atIndex:1];
    [encoder setBuffer:buf_result offset:0 atIndex:2];
    [encoder setBytes:&n length:sizeof(int) atIndex:3];
    [encoder setBytes:&m length:sizeof(int) atIndex:4];

    MTLSize grid = MTLSizeMake(n, m, 1);
    MTLSize group = MTLSizeMake(16, 16, 1);

    [encoder dispatchThreads:grid threadsPerThreadgroup:group];
    [encoder endEncoding];
    [cmd commit];
    [cmd waitUntilCompleted];

    Poly result(result_size);
    memcpy(result.data(), [buf_result contents], result_size * sizeof(int));
    return result;
  }
};

bool verify(const Poly &a, const Poly &b) {
  if (a.size() != b.size())
    return false;
  for (size_t i = 0; i < a.size(); i++)
    if (a[i] != b[i])
      return false;
  return true;
}

template <typename F> double bench(F &&f) {
  auto start = Clock::now();
  f();
  auto end = Clock::now();
  return std::chrono::duration<double, std::milli>(end - start).count();
}

void run_test(MetalCompute &metal, const char *name, const Poly &p1,
              const Poly &p2) {
  printf("=== Test: %s ===\n", name);
  printf("polynomial size: %zu x %zu\n\n", p1.size(), p2.size());

  Poly cpu_result, karatsuba_result, metal_regular_result, metal_tiled_result;

  double cpu_time = bench([&] { cpu_result = multiply_cpu(p1, p2); });
  double karatsuba_time = bench([&] { karatsuba_result = karatsuba(p1, p2); });
  double metal_regular_time =
      bench([&] { metal_regular_result = metal.multiply_regular(p1, p2); });
  double metal_tiled_time =
      bench([&] { metal_tiled_result = metal.multiply_tiled(p1, p2); });

  bool ok = verify(cpu_result, karatsuba_result) &&
            verify(cpu_result, metal_regular_result) &&
            verify(cpu_result, metal_tiled_result);

  printf("results: %s\n\n", ok ? "CORRECT" : "WRONG");

  printf("first 5 result coeffs: ");
  for (int i = 0; i < 5 && i < (int)cpu_result.size(); i++)
    printf("%d ", cpu_result[i]);
  printf("\nlast 5 result coeffs:  ");
  for (int i = std::max(0, (int)cpu_result.size() - 5);
       i < (int)cpu_result.size(); i++)
    printf("%d ", cpu_result[i]);
  printf("\n\n");

  printf("%-25s %10.2f ms  %6.2fx\n", "cpu O(n^2)", cpu_time, 1.0);
  printf("%-25s %10.2f ms  %6.2fx\n", "cpu karatsuba", karatsuba_time,
         cpu_time / karatsuba_time);
  printf("%-25s %10.2f ms  %6.2fx\n", "metal regular", metal_regular_time,
         cpu_time / metal_regular_time);
  printf("%-25s %10.2f ms  %6.2fx\n", "metal tiled", metal_tiled_time,
         cpu_time / metal_tiled_time);
  printf("\n");
}

int main(int argc, char **argv) {
  MetalCompute metal;

  {
    Poly p1 = ones_poly(1000);
    Poly p2 = ones_poly(1000);
    run_test(metal, "All-ones 1000 coefficients", p1, p2);

    Poly expected(1999);
    for (int k = 0; k < 1999; k++) {
      expected[k] = std::min(k + 1, 1999 - k);
    }
    Poly actual = multiply_cpu(p1, p2);
    bool pattern_ok = verify(actual, expected);
    printf("all-ones pattern verification: %s\n",
           pattern_ok ? "CORRECT" : "WRONG");
    printf("expected: 1,2,3,...,1000,...,3,2,1\n");
    printf("got:      %d,%d,%d,...,%d,...,%d,%d,%d\n\n", actual[0], actual[1],
           actual[2], actual[999], actual[1996], actual[1997], actual[1998]);
  }

  {
    Poly p1 = random_poly(10000, 42);
    Poly p2 = random_poly(10000, 123);
    run_test(metal, "Random 10000 coefficients", p1, p2);
  }

  {
    Poly p1 = random_poly(50000, 42);
    Poly p2 = random_poly(50000, 123);
    run_test(metal, "Random 50000 coefficients", p1, p2);
  }

  return 0;
}
