# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/workspace/jazzy_ws/src/ament_cmake/ament_cmake_vendor_package/test/depender"
  "/workspace/jazzy_ws/build/ament_cmake_vendor_package/test/depender-prefix/src/depender-build"
  "/workspace/jazzy_ws/build/ament_cmake_vendor_package/test/depender-prefix/install"
  "/workspace/jazzy_ws/build/ament_cmake_vendor_package/test/depender-prefix/tmp"
  "/workspace/jazzy_ws/build/ament_cmake_vendor_package/test/depender-prefix/src/depender-stamp"
  "/workspace/jazzy_ws/build/ament_cmake_vendor_package/test/depender-prefix/src"
  "/workspace/jazzy_ws/build/ament_cmake_vendor_package/test/depender-prefix/src/depender-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/workspace/jazzy_ws/build/ament_cmake_vendor_package/test/depender-prefix/src/depender-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/workspace/jazzy_ws/build/ament_cmake_vendor_package/test/depender-prefix/src/depender-stamp${cfgdir}") # cfgdir has leading slash
endif()
