cmake_minimum_required(VERSION 3.16)
project(robot_bang_b_logic LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(robot_logic_test tests/robot_logic_test.cpp)
target_compile_options(robot_logic_test PRIVATE -Wall -Wextra -Wpedantic)

enable_testing()
add_test(NAME robot_logic_test COMMAND robot_logic_test)
