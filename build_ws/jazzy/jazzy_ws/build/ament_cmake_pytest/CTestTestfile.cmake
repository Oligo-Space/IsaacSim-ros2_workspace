# CMake generated Testfile for 
# Source directory: /workspace/jazzy_ws/src/ament_cmake/ament_cmake_pytest
# Build directory: /workspace/jazzy_ws/build/ament_cmake_pytest
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(pytest "/usr/bin/python3" "-u" "/workspace/jazzy_ws/install/share/ament_cmake_test/cmake/run_test.py" "/workspace/jazzy_ws/build/ament_cmake_pytest/test_results/ament_cmake_pytest/pytest.xunit.xml" "--package-name" "ament_cmake_pytest" "--output-file" "/workspace/jazzy_ws/build/ament_cmake_pytest/ament_cmake_pytest/pytest.txt" "--command" "/usr/bin/python3" "-u" "-m" "pytest" "/workspace/jazzy_ws/src/ament_cmake/ament_cmake_pytest/test" "-o" "cache_dir=/workspace/jazzy_ws/build/ament_cmake_pytest/ament_cmake_pytest/pytest/.cache" "-s" "--junit-xml=/workspace/jazzy_ws/build/ament_cmake_pytest/test_results/ament_cmake_pytest/pytest.xunit.xml" "--junit-prefix=ament_cmake_pytest")
set_tests_properties(pytest PROPERTIES  LABELS "pytest" TIMEOUT "60" WORKING_DIRECTORY "/workspace/jazzy_ws/build/ament_cmake_pytest" _BACKTRACE_TRIPLES "/workspace/jazzy_ws/install/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/workspace/jazzy_ws/src/ament_cmake/ament_cmake_pytest/CMakeLists.txt;50;ament_add_test;/workspace/jazzy_ws/src/ament_cmake/ament_cmake_pytest/CMakeLists.txt;0;")
