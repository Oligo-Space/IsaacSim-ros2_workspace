# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

if("/workspace/jazzy_ws/src/osrf_testing_tools_cpp/vendor/google/googletest/release-1.14.0.tar.gz" STREQUAL "")
  message(FATAL_ERROR "LOCAL can't be empty")
endif()

if(NOT EXISTS "/workspace/jazzy_ws/src/osrf_testing_tools_cpp/vendor/google/googletest/release-1.14.0.tar.gz")
  message(FATAL_ERROR "File not found: /workspace/jazzy_ws/src/osrf_testing_tools_cpp/vendor/google/googletest/release-1.14.0.tar.gz")
endif()

if("MD5" STREQUAL "")
  message(WARNING "File will not be verified since no URL_HASH specified")
  return()
endif()

if("c8340a482851ef6a3fe618a082304cfc" STREQUAL "")
  message(FATAL_ERROR "EXPECT_VALUE can't be empty")
endif()

message(STATUS "verifying file...
     file='/workspace/jazzy_ws/src/osrf_testing_tools_cpp/vendor/google/googletest/release-1.14.0.tar.gz'")

file("MD5" "/workspace/jazzy_ws/src/osrf_testing_tools_cpp/vendor/google/googletest/release-1.14.0.tar.gz" actual_value)

if(NOT "${actual_value}" STREQUAL "c8340a482851ef6a3fe618a082304cfc")
  message(FATAL_ERROR "error: MD5 hash of
  /workspace/jazzy_ws/src/osrf_testing_tools_cpp/vendor/google/googletest/release-1.14.0.tar.gz
does not match expected value
  expected: 'c8340a482851ef6a3fe618a082304cfc'
    actual: '${actual_value}'
")
endif()

message(STATUS "verifying file... done")
