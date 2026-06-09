# Install script for directory: /workspace/jazzy_ws/build/mimick_vendor/mimick_vendor-prefix/src/mimick_vendor

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/workspace/jazzy_ws/build/mimick_vendor/mimick_vendor-prefix/install")
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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/mimick" TYPE FILE FILES "/workspace/jazzy_ws/build/mimick_vendor/mimick_vendor-prefix/src/mimick_vendor/include/mimick/alloc.h")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/mimick" TYPE FILE FILES "/workspace/jazzy_ws/build/mimick_vendor/mimick_vendor-prefix/src/mimick_vendor/include/mimick/assert.h")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/mimick" TYPE FILE FILES "/workspace/jazzy_ws/build/mimick_vendor/mimick_vendor-prefix/src/mimick_vendor/include/mimick/literal.h")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/mimick" TYPE FILE FILES "/workspace/jazzy_ws/build/mimick_vendor/mimick_vendor-prefix/src/mimick_vendor/include/mimick/matcher.h")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/mimick" TYPE FILE FILES "/workspace/jazzy_ws/build/mimick_vendor/mimick_vendor-prefix/src/mimick_vendor/include/mimick/mock.h")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/mimick" TYPE FILE FILES "/workspace/jazzy_ws/build/mimick_vendor/mimick_vendor-prefix/src/mimick_vendor/include/mimick/preprocess.h")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/mimick" TYPE FILE FILES "/workspace/jazzy_ws/build/mimick_vendor/mimick_vendor-prefix/src/mimick_vendor/include/mimick/string.h")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/mimick" TYPE FILE FILES "/workspace/jazzy_ws/build/mimick_vendor/mimick_vendor-prefix/src/mimick_vendor/include/mimick/unmocked.h")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/mimick" TYPE FILE FILES "/workspace/jazzy_ws/build/mimick_vendor/mimick_vendor-prefix/src/mimick_vendor/include/mimick/va.h")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/mimick" TYPE FILE FILES "/workspace/jazzy_ws/build/mimick_vendor/mimick_vendor-prefix/src/mimick_vendor/include/mimick/verify.h")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/mimick" TYPE FILE FILES "/workspace/jazzy_ws/build/mimick_vendor/mimick_vendor-prefix/src/mimick_vendor/include/mimick/when.h")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/mimick" TYPE FILE FILES "/workspace/jazzy_ws/build/mimick_vendor/mimick_vendor-prefix/src/mimick_vendor/include/mimick/mimick.h")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "/workspace/jazzy_ws/build/mimick_vendor/mimick_vendor-prefix/src/mimick_vendor-build/libmimick.a")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/mimick/mimick-targets.cmake")
    file(DIFFERENT _cmake_export_file_changed FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/mimick/mimick-targets.cmake"
         "/workspace/jazzy_ws/build/mimick_vendor/mimick_vendor-prefix/src/mimick_vendor-build/CMakeFiles/Export/ac0f55e42a8f5b6fc2e1d2d6a491a792/mimick-targets.cmake")
    if(_cmake_export_file_changed)
      file(GLOB _cmake_old_config_files "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/mimick/mimick-targets-*.cmake")
      if(_cmake_old_config_files)
        string(REPLACE ";" ", " _cmake_old_config_files_text "${_cmake_old_config_files}")
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/mimick/mimick-targets.cmake\" will be replaced.  Removing files [${_cmake_old_config_files_text}].")
        unset(_cmake_old_config_files_text)
        file(REMOVE ${_cmake_old_config_files})
      endif()
      unset(_cmake_old_config_files)
    endif()
    unset(_cmake_export_file_changed)
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/mimick" TYPE FILE FILES "/workspace/jazzy_ws/build/mimick_vendor/mimick_vendor-prefix/src/mimick_vendor-build/CMakeFiles/Export/ac0f55e42a8f5b6fc2e1d2d6a491a792/mimick-targets.cmake")
  if(CMAKE_INSTALL_CONFIG_NAME MATCHES "^()$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/mimick" TYPE FILE FILES "/workspace/jazzy_ws/build/mimick_vendor/mimick_vendor-prefix/src/mimick_vendor-build/CMakeFiles/Export/ac0f55e42a8f5b6fc2e1d2d6a491a792/mimick-targets-noconfig.cmake")
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/mimick" TYPE FILE FILES "/workspace/jazzy_ws/build/mimick_vendor/mimick_vendor-prefix/src/mimick_vendor-build/mimickConfig.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/workspace/jazzy_ws/build/mimick_vendor/mimick_vendor-prefix/src/mimick_vendor-build/src/cmake_install.cmake")
  include("/workspace/jazzy_ws/build/mimick_vendor/mimick_vendor-prefix/src/mimick_vendor-build/test/cmake_install.cmake")
  include("/workspace/jazzy_ws/build/mimick_vendor/mimick_vendor-prefix/src/mimick_vendor-build/sample/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/workspace/jazzy_ws/build/mimick_vendor/mimick_vendor-prefix/src/mimick_vendor-build/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
