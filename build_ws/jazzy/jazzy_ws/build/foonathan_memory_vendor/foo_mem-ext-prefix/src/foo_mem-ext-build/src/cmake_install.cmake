# Install script for directory: /workspace/jazzy_ws/build/foonathan_memory_vendor/foo_mem-ext-prefix/src/foo_mem-ext/src

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/workspace/jazzy_ws/build/foonathan_memory_vendor/foo_mem_ext_prj_install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "/workspace/jazzy_ws/build/foonathan_memory_vendor/foo_mem-ext-prefix/src/foo_mem-ext-build/src/libfoonathan_memory-0.7.3.a")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/foonathan_memory/foonathan/memory" TYPE FILE FILES "/workspace/jazzy_ws/build/foonathan_memory_vendor/foo_mem-ext-prefix/src/foo_mem-ext-build/src/config_impl.hpp")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/foonathan_memory/foonathan/memory/detail" TYPE FILE FILES "/workspace/jazzy_ws/build/foonathan_memory_vendor/foo_mem-ext-prefix/src/foo_mem-ext-build/src/container_node_sizes_impl.hpp")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/foonathan_memory/foonathan/memory" TYPE FILE FILES
    "/workspace/jazzy_ws/build/foonathan_memory_vendor/foo_mem-ext-prefix/src/foo_mem-ext/include/foonathan/memory/aligned_allocator.hpp"
    "/workspace/jazzy_ws/build/foonathan_memory_vendor/foo_mem-ext-prefix/src/foo_mem-ext/include/foonathan/memory/allocator_storage.hpp"
    "/workspace/jazzy_ws/build/foonathan_memory_vendor/foo_mem-ext-prefix/src/foo_mem-ext/include/foonathan/memory/allocator_traits.hpp"
    "/workspace/jazzy_ws/build/foonathan_memory_vendor/foo_mem-ext-prefix/src/foo_mem-ext/include/foonathan/memory/config.hpp"
    "/workspace/jazzy_ws/build/foonathan_memory_vendor/foo_mem-ext-prefix/src/foo_mem-ext/include/foonathan/memory/container.hpp"
    "/workspace/jazzy_ws/build/foonathan_memory_vendor/foo_mem-ext-prefix/src/foo_mem-ext/include/foonathan/memory/debugging.hpp"
    "/workspace/jazzy_ws/build/foonathan_memory_vendor/foo_mem-ext-prefix/src/foo_mem-ext/include/foonathan/memory/default_allocator.hpp"
    "/workspace/jazzy_ws/build/foonathan_memory_vendor/foo_mem-ext-prefix/src/foo_mem-ext/include/foonathan/memory/deleter.hpp"
    "/workspace/jazzy_ws/build/foonathan_memory_vendor/foo_mem-ext-prefix/src/foo_mem-ext/include/foonathan/memory/error.hpp"
    "/workspace/jazzy_ws/build/foonathan_memory_vendor/foo_mem-ext-prefix/src/foo_mem-ext/include/foonathan/memory/fallback_allocator.hpp"
    "/workspace/jazzy_ws/build/foonathan_memory_vendor/foo_mem-ext-prefix/src/foo_mem-ext/include/foonathan/memory/malloc_allocator.hpp"
    "/workspace/jazzy_ws/build/foonathan_memory_vendor/foo_mem-ext-prefix/src/foo_mem-ext/include/foonathan/memory/heap_allocator.hpp"
    "/workspace/jazzy_ws/build/foonathan_memory_vendor/foo_mem-ext-prefix/src/foo_mem-ext/include/foonathan/memory/iteration_allocator.hpp"
    "/workspace/jazzy_ws/build/foonathan_memory_vendor/foo_mem-ext-prefix/src/foo_mem-ext/include/foonathan/memory/joint_allocator.hpp"
    "/workspace/jazzy_ws/build/foonathan_memory_vendor/foo_mem-ext-prefix/src/foo_mem-ext/include/foonathan/memory/memory_arena.hpp"
    "/workspace/jazzy_ws/build/foonathan_memory_vendor/foo_mem-ext-prefix/src/foo_mem-ext/include/foonathan/memory/memory_pool.hpp"
    "/workspace/jazzy_ws/build/foonathan_memory_vendor/foo_mem-ext-prefix/src/foo_mem-ext/include/foonathan/memory/memory_pool_collection.hpp"
    "/workspace/jazzy_ws/build/foonathan_memory_vendor/foo_mem-ext-prefix/src/foo_mem-ext/include/foonathan/memory/memory_pool_type.hpp"
    "/workspace/jazzy_ws/build/foonathan_memory_vendor/foo_mem-ext-prefix/src/foo_mem-ext/include/foonathan/memory/memory_resource_adapter.hpp"
    "/workspace/jazzy_ws/build/foonathan_memory_vendor/foo_mem-ext-prefix/src/foo_mem-ext/include/foonathan/memory/memory_stack.hpp"
    "/workspace/jazzy_ws/build/foonathan_memory_vendor/foo_mem-ext-prefix/src/foo_mem-ext/include/foonathan/memory/namespace_alias.hpp"
    "/workspace/jazzy_ws/build/foonathan_memory_vendor/foo_mem-ext-prefix/src/foo_mem-ext/include/foonathan/memory/new_allocator.hpp"
    "/workspace/jazzy_ws/build/foonathan_memory_vendor/foo_mem-ext-prefix/src/foo_mem-ext/include/foonathan/memory/segregator.hpp"
    "/workspace/jazzy_ws/build/foonathan_memory_vendor/foo_mem-ext-prefix/src/foo_mem-ext/include/foonathan/memory/smart_ptr.hpp"
    "/workspace/jazzy_ws/build/foonathan_memory_vendor/foo_mem-ext-prefix/src/foo_mem-ext/include/foonathan/memory/static_allocator.hpp"
    "/workspace/jazzy_ws/build/foonathan_memory_vendor/foo_mem-ext-prefix/src/foo_mem-ext/include/foonathan/memory/std_allocator.hpp"
    "/workspace/jazzy_ws/build/foonathan_memory_vendor/foo_mem-ext-prefix/src/foo_mem-ext/include/foonathan/memory/temporary_allocator.hpp"
    "/workspace/jazzy_ws/build/foonathan_memory_vendor/foo_mem-ext-prefix/src/foo_mem-ext/include/foonathan/memory/threading.hpp"
    "/workspace/jazzy_ws/build/foonathan_memory_vendor/foo_mem-ext-prefix/src/foo_mem-ext/include/foonathan/memory/tracking.hpp"
    "/workspace/jazzy_ws/build/foonathan_memory_vendor/foo_mem-ext-prefix/src/foo_mem-ext/include/foonathan/memory/virtual_memory.hpp"
    "/workspace/jazzy_ws/build/foonathan_memory_vendor/foo_mem-ext-prefix/src/foo_mem-ext-build/src/container_node_sizes_impl.hpp"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/foonathan_memory/foonathan/memory/detail" TYPE FILE FILES
    "/workspace/jazzy_ws/build/foonathan_memory_vendor/foo_mem-ext-prefix/src/foo_mem-ext/include/foonathan/memory/detail/align.hpp"
    "/workspace/jazzy_ws/build/foonathan_memory_vendor/foo_mem-ext-prefix/src/foo_mem-ext/include/foonathan/memory/detail/assert.hpp"
    "/workspace/jazzy_ws/build/foonathan_memory_vendor/foo_mem-ext-prefix/src/foo_mem-ext/include/foonathan/memory/detail/container_node_sizes.hpp"
    "/workspace/jazzy_ws/build/foonathan_memory_vendor/foo_mem-ext-prefix/src/foo_mem-ext/include/foonathan/memory/detail/debug_helpers.hpp"
    "/workspace/jazzy_ws/build/foonathan_memory_vendor/foo_mem-ext-prefix/src/foo_mem-ext/include/foonathan/memory/detail/ebo_storage.hpp"
    "/workspace/jazzy_ws/build/foonathan_memory_vendor/foo_mem-ext-prefix/src/foo_mem-ext/include/foonathan/memory/detail/free_list.hpp"
    "/workspace/jazzy_ws/build/foonathan_memory_vendor/foo_mem-ext-prefix/src/foo_mem-ext/include/foonathan/memory/detail/free_list_array.hpp"
    "/workspace/jazzy_ws/build/foonathan_memory_vendor/foo_mem-ext-prefix/src/foo_mem-ext/include/foonathan/memory/detail/ilog2.hpp"
    "/workspace/jazzy_ws/build/foonathan_memory_vendor/foo_mem-ext-prefix/src/foo_mem-ext/include/foonathan/memory/detail/lowlevel_allocator.hpp"
    "/workspace/jazzy_ws/build/foonathan_memory_vendor/foo_mem-ext-prefix/src/foo_mem-ext/include/foonathan/memory/detail/memory_stack.hpp"
    "/workspace/jazzy_ws/build/foonathan_memory_vendor/foo_mem-ext-prefix/src/foo_mem-ext/include/foonathan/memory/detail/small_free_list.hpp"
    "/workspace/jazzy_ws/build/foonathan_memory_vendor/foo_mem-ext-prefix/src/foo_mem-ext/include/foonathan/memory/detail/utility.hpp"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/foonathan_memory/cmake" TYPE FILE FILES "/workspace/jazzy_ws/build/foonathan_memory_vendor/foo_mem-ext-prefix/src/foo_mem-ext-build/src/cmake/foonathan_memory-config-version.cmake")
endif()

