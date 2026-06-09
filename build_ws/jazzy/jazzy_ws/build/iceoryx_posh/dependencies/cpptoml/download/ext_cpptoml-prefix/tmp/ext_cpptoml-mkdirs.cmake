# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/workspace/jazzy_ws/build/iceoryx_posh/dependencies/cpptoml/src"
  "/workspace/jazzy_ws/build/iceoryx_posh/dependencies/cpptoml/build"
  "/workspace/jazzy_ws/build/iceoryx_posh/dependencies/cpptoml/download/ext_cpptoml-prefix"
  "/workspace/jazzy_ws/build/iceoryx_posh/dependencies/cpptoml/download/ext_cpptoml-prefix/tmp"
  "/workspace/jazzy_ws/build/iceoryx_posh/dependencies/cpptoml/download/ext_cpptoml-prefix/src/ext_cpptoml-stamp"
  "/workspace/jazzy_ws/build/iceoryx_posh/dependencies/cpptoml/download/ext_cpptoml-prefix/src"
  "/workspace/jazzy_ws/build/iceoryx_posh/dependencies/cpptoml/download/ext_cpptoml-prefix/src/ext_cpptoml-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/workspace/jazzy_ws/build/iceoryx_posh/dependencies/cpptoml/download/ext_cpptoml-prefix/src/ext_cpptoml-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/workspace/jazzy_ws/build/iceoryx_posh/dependencies/cpptoml/download/ext_cpptoml-prefix/src/ext_cpptoml-stamp${cfgdir}") # cfgdir has leading slash
endif()
