# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/workspace/jazzy_ws/build/google_benchmark_vendor/benchmark-1.8.3-prefix/src/benchmark-1.8.3"
  "/workspace/jazzy_ws/build/google_benchmark_vendor/benchmark-1.8.3-prefix/src/benchmark-1.8.3-build"
  "/workspace/jazzy_ws/build/google_benchmark_vendor/benchmark-1.8.3-prefix"
  "/workspace/jazzy_ws/build/google_benchmark_vendor/benchmark-1.8.3-prefix/tmp"
  "/workspace/jazzy_ws/build/google_benchmark_vendor/benchmark-1.8.3-prefix/src/benchmark-1.8.3-stamp"
  "/workspace/jazzy_ws/build/google_benchmark_vendor/benchmark-1.8.3-prefix/src"
  "/workspace/jazzy_ws/build/google_benchmark_vendor/benchmark-1.8.3-prefix/src/benchmark-1.8.3-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/workspace/jazzy_ws/build/google_benchmark_vendor/benchmark-1.8.3-prefix/src/benchmark-1.8.3-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/workspace/jazzy_ws/build/google_benchmark_vendor/benchmark-1.8.3-prefix/src/benchmark-1.8.3-stamp${cfgdir}") # cfgdir has leading slash
endif()
