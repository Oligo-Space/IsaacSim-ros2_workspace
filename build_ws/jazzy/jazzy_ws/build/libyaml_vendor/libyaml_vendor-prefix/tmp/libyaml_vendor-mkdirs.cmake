# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/workspace/jazzy_ws/build/libyaml_vendor/libyaml_vendor-prefix/src/libyaml_vendor"
  "/workspace/jazzy_ws/build/libyaml_vendor/libyaml_vendor-prefix/src/libyaml_vendor-build"
  "/workspace/jazzy_ws/build/libyaml_vendor/libyaml_vendor-prefix/install"
  "/workspace/jazzy_ws/build/libyaml_vendor/libyaml_vendor-prefix/tmp"
  "/workspace/jazzy_ws/build/libyaml_vendor/libyaml_vendor-prefix/src/libyaml_vendor-stamp"
  "/workspace/jazzy_ws/build/libyaml_vendor/libyaml_vendor-prefix/src"
  "/workspace/jazzy_ws/build/libyaml_vendor/libyaml_vendor-prefix/src/libyaml_vendor-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/workspace/jazzy_ws/build/libyaml_vendor/libyaml_vendor-prefix/src/libyaml_vendor-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/workspace/jazzy_ws/build/libyaml_vendor/libyaml_vendor-prefix/src/libyaml_vendor-stamp${cfgdir}") # cfgdir has leading slash
endif()
