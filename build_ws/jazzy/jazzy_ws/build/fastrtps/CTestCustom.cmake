find_package(Python3 COMPONENTS Interpreter REQUIRED)

execute_process(COMMAND  /workspace/jazzy_ws/src/fastrtps/test/../tools/fastdds/fastdds.py shm clean)
