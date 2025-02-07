// SPDX-License-Identifier: MIT
// Copyright (c) 2021  Kenji Koide (k.koide@aist.go.jp)
#include <gtsam_points/cuda/cuda_memory.hpp>

#include <cuda_runtime.h>
#include <gtsam_points/cuda/check_error.cuh>

namespace gtsam_points {

void *cuda_malloc(size_t size, CUstream_st *stream) {
  void *ptr = nullptr;
#if CUDAToolkit_VERSION_MAJOR == 11
#if CUDAToolkit_VERSION_MINOR >= 4
  check_error << cudaMallocAsync(&ptr, size, stream);
#else
  check_error << cudaMalloc(&ptr, size);
#endif
#endif
  return ptr;
}

void *cuda_malloc_and_upload(const void *data, size_t size,
                             CUstream_st *stream) {
  void *ptr = nullptr;

#if CUDAToolkit_VERSION_MAJOR == 11
#if CUDAToolkit_VERSION_MINOR >= 4
  check_error << cudaMallocAsync(&ptr, size, stream);
#else
  check_error << cudaMalloc(&ptr, size);
#endif
#endif

  check_error << cudaMemcpyAsync(ptr, data, size, cudaMemcpyHostToDevice,
                                 stream);
  return ptr;
}

void cuda_free(void *ptr, CUstream_st *stream) {
#if CUDAToolkit_VERSION_MAJOR == 11
#if CUDAToolkit_VERSION_MINOR >= 4
  check_error << cudaFreeAsync(ptr, stream);
#else
  check_error << cudaFree(ptr);
#endif
#endif
}

void cuda_host_to_device(void *dst, const void *src, size_t size,
                         CUstream_st *stream) {
  check_error << cudaMemcpyAsync(dst, src, size, cudaMemcpyHostToDevice,
                                 stream);
}

void cuda_device_to_host(void *dst, const void *src, size_t size,
                         CUstream_st *stream) {
  check_error << cudaMemcpyAsync(dst, src, size, cudaMemcpyDeviceToHost,
                                 stream);
}

void cuda_mem_get_info(size_t *free, size_t *total) {
  check_error << cudaMemGetInfo(free, total);
}

} // namespace gtsam_points
