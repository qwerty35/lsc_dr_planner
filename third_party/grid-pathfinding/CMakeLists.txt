cmake_minimum_required(VERSION 3.16)
project(graph CXX)

add_subdirectory(./graph)
add_subdirectory(./third_party/googletest)

# format
add_custom_target(clang-format
  COMMAND clang-format -i
  ../graph/include/*.hpp
  ../graph/src/*.cpp
  ../tests/*.cpp)

# test
set(TEST_MAIN_FUNC ./third_party/googletest/googletest/src/gtest_main.cc)
add_executable(test ./tests/test_graph.cpp ${TEST_ALL_SRC} ${TEST_MAIN_FUNC})
target_link_libraries(test lib-graph gtest)
