#include <gtest/gtest.h>
#include <vector>
#include <random>
#include <algorithm>
#include <iostream>
#include <chrono>
#include <rs_driver/utility/simd.hpp>

using namespace robosense::lidar;

TEST(Copy3of4BytesTest, HandlesEmptyInput)
{
  std::vector<uint8_t> src;
  std::vector<uint8_t> dst(1);

  size_t result_size = copy_3of4_bytes_optimized(src.data(), dst.data(), src.size());

  EXPECT_EQ(result_size, 0);
}

TEST(Copy3of4BytesTest, CopiesSingleGroup)
{
  std::vector<uint8_t> src = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B,
                               0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16,
                               0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F, 0x20 };

  std::vector<uint8_t> expected_dst = { 0x03, 0x02, 0x01, 0x07, 0x06, 0x05, 0x0B, 0x0A, 0x09, 0x0F, 0x0E, 0x0D,
                                        0x13, 0x12, 0x11, 0x17, 0x16, 0x15, 0x1B, 0x1A, 0x19, 0x1F, 0x1E, 0x1D };

  std::vector<uint8_t> actual_dst(expected_dst.size());

  size_t result_size = copy_3of4_bytes_optimized(src.data(), actual_dst.data(), src.size());

  EXPECT_EQ(result_size, expected_dst.size());
  EXPECT_EQ(actual_dst, expected_dst);
}

TEST(Copy3of4BytesTest, MatchesGenericVersionOnRandomData)
{
  const size_t test_size = 6400 * 2400;
  std::vector<uint8_t> src(test_size);

  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<> dis(0, 255);
  std::generate(src.begin(), src.end(), [&]() { return static_cast<uint8_t>(dis(gen)); });

  size_t expected_dst_size = 4800 * 2400;
  std::vector<uint8_t> dst_generic(expected_dst_size);
  std::vector<uint8_t> dst_optimized(expected_dst_size);

  copy_3of4_bytes_generic(src.data(), dst_generic.data(), src.size());
  copy_3of4_bytes_optimized(src.data(), dst_optimized.data(), src.size());

  EXPECT_EQ(dst_generic, dst_optimized) << "Optimized version does not match generic version.";
}

TEST(Copy3of4BytesBenchmark, PerformanceComparison)
{
  const size_t test_size = 6400 * 2400;
  const size_t num_iterations = 100;  // 减少迭代次数，除非你想长时间运行
  std::vector<uint8_t> src(test_size);
  std::vector<uint8_t> dst_generic(4800 * 2400);
  std::vector<uint8_t> dst_optimized(dst_generic.size());

  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<> dis(0, 255);
  std::generate(src.begin(), src.end(), [&]() { return static_cast<uint8_t>(dis(gen)); });

  auto start = std::chrono::high_resolution_clock::now();
  for (size_t i = 0; i < num_iterations; ++i)
  {
    copy_3of4_bytes_generic(src.data(), dst_generic.data(), src.size());
  }
  auto end_generic = std::chrono::high_resolution_clock::now();
  auto duration_generic = std::chrono::duration_cast<std::chrono::microseconds>(end_generic - start).count();

  start = std::chrono::high_resolution_clock::now();
  for (size_t i = 0; i < num_iterations; ++i)
  {
    copy_3of4_bytes_optimized(src.data(), dst_optimized.data(), src.size());
  }
  auto end_optimized = std::chrono::high_resolution_clock::now();
  auto duration_optimized = std::chrono::duration_cast<std::chrono::microseconds>(end_optimized - start).count();

  std::cout << "\n[BENCHMARK] Generic version took: " << duration_generic << " μs for " << num_iterations << " runs.\n";
  std::cout << "[BENCHMARK] Optimized version took: " << duration_optimized << " μs for " << num_iterations
            << " runs.\n";

  if (duration_optimized > 0)
  {
    double speedup = static_cast<double>(duration_generic) / duration_optimized;
    std::cout << "[BENCHMARK] Speedup: " << std::fixed << std::setprecision(2) << speedup << "x\n";
  }
}
