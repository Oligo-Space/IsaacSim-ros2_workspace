# CMake configuration for spdlog_vendor
set(CMAKE_CXX_COMPILER [=[/usr/bin/c++]=] CACHE INTERNAL "")
set(CMAKE_CXX_FLAGS [=[-Wno-deprecated-declarations ]=] CACHE INTERNAL "")
set(CMAKE_VERBOSE_MAKEFILE [=[FALSE]=] CACHE INTERNAL "")
set(CMAKE_BUILD_TYPE [=[]=] CACHE INTERNAL "")
set(CMAKE_PREFIX_PATH [=[/workspace/jazzy_ws/build/spdlog_vendor/spdlog_vendor-prefix/install]=] CACHE INTERNAL "")
set(BUILD_TESTING "OFF" CACHE INTERNAL "")
set(BUILD_SHARED_LIBS [=[TRUE]=] CACHE INTERNAL "")