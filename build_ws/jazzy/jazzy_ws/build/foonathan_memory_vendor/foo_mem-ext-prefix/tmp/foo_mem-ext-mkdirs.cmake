# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/workspace/jazzy_ws/build/foonathan_memory_vendor/foo_mem-ext-prefix/src/foo_mem-ext"
  "/workspace/jazzy_ws/build/foonathan_memory_vendor/foo_mem-ext-prefix/src/foo_mem-ext-build"
  "/workspace/jazzy_ws/build/foonathan_memory_vendor/foo_mem-ext-prefix"
  "/workspace/jazzy_ws/build/foonathan_memory_vendor/foo_mem-ext-prefix/tmp"
  "/workspace/jazzy_ws/build/foonathan_memory_vendor/foo_mem-ext-prefix/src/foo_mem-ext-stamp"
  "/workspace/jazzy_ws/build/foonathan_memory_vendor/foo_mem-ext-prefix/src"
  "/workspace/jazzy_ws/build/foonathan_memory_vendor/foo_mem-ext-prefix/src/foo_mem-ext-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/workspace/jazzy_ws/build/foonathan_memory_vendor/foo_mem-ext-prefix/src/foo_mem-ext-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/workspace/jazzy_ws/build/foonathan_memory_vendor/foo_mem-ext-prefix/src/foo_mem-ext-stamp${cfgdir}") # cfgdir has leading slash
endif()
