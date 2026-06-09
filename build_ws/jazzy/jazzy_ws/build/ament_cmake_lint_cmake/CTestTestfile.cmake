# CMake generated Testfile for 
# Source directory: /workspace/jazzy_ws/src/ament_lint/ament_cmake_lint_cmake
# Build directory: /workspace/jazzy_ws/build/ament_cmake_lint_cmake
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(lint_cmake "/usr/bin/python3" "-u" "/workspace/jazzy_ws/install/share/ament_cmake_test/cmake/run_test.py" "/workspace/jazzy_ws/build/ament_cmake_lint_cmake/test_results/ament_cmake_lint_cmake/lint_cmake.xunit.xml" "--package-name" "ament_cmake_lint_cmake" "--output-file" "/workspace/jazzy_ws/build/ament_cmake_lint_cmake/ament_lint_cmake/lint_cmake.txt" "--command" "/workspace/jazzy_ws/install/bin/ament_lint_cmake" "--xunit-file" "/workspace/jazzy_ws/build/ament_cmake_lint_cmake/test_results/ament_cmake_lint_cmake/lint_cmake.xunit.xml")
set_tests_properties(lint_cmake PROPERTIES  LABELS "lint_cmake;linter" TIMEOUT "60" WORKING_DIRECTORY "/workspace/jazzy_ws/src/ament_lint/ament_cmake_lint_cmake" _BACKTRACE_TRIPLES "/workspace/jazzy_ws/install/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/workspace/jazzy_ws/src/ament_lint/ament_cmake_lint_cmake/cmake/ament_lint_cmake.cmake;47;ament_add_test;/workspace/jazzy_ws/src/ament_lint/ament_cmake_lint_cmake/CMakeLists.txt;22;ament_lint_cmake;/workspace/jazzy_ws/src/ament_lint/ament_cmake_lint_cmake/CMakeLists.txt;0;")
