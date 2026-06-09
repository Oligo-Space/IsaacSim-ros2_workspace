# CMake generated Testfile for 
# Source directory: /workspace/jazzy_ws/src/osrf_testing_tools_cpp/test/test_runner
# Build directory: /workspace/jazzy_ws/build/osrf_testing_tools_cpp/test/test_runner
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(test_parse_environment_variable "/workspace/jazzy_ws/build/osrf_testing_tools_cpp/test/test_runner/test_parse_environment_variable")
set_tests_properties(test_parse_environment_variable PROPERTIES  _BACKTRACE_TRIPLES "/workspace/jazzy_ws/src/osrf_testing_tools_cpp/test/test_runner/CMakeLists.txt;5;add_test;/workspace/jazzy_ws/src/osrf_testing_tools_cpp/test/test_runner/CMakeLists.txt;0;")
add_test(test_test_runner "/workspace/jazzy_ws/build/osrf_testing_tools_cpp/src/test_runner/test_runner" "--env" "FOO=bar" "PING=pong" "--append-env" "FOO=baz" "--" "/workspace/jazzy_ws/build/osrf_testing_tools_cpp/test/test_runner/assert_env_vars" "--env" "FOO=bar:baz" "PING=pong")
set_tests_properties(test_test_runner PROPERTIES  _BACKTRACE_TRIPLES "/workspace/jazzy_ws/src/osrf_testing_tools_cpp/test/test_runner/CMakeLists.txt;16;add_test;/workspace/jazzy_ws/src/osrf_testing_tools_cpp/test/test_runner/CMakeLists.txt;0;")
