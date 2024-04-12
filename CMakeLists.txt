cmake_minimum_required(VERSION 3.14)
set(CMAKE_FIND_DEBUG_MODE 0)
project(1x VERSION 1.0)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

find_package(Eigen3 3.3 REQUIRED)
find_package(GTest CONFIG REQUIRED)
find_package(absl REQUIRED)

# Library
set(SRCS
  ${CMAKE_CURRENT_SOURCE_DIR}/src/robot.cc
)
set(HDRS
  ${CMAKE_CURRENT_SOURCE_DIR}/include/robot.h
)
add_library(robot)
target_sources(robot PUBLIC ${HDRS} PRIVATE ${SRCS})
target_link_libraries(robot
  PUBLIC
    absl::strings
    absl::span
    absl::algorithm
  PRIVATE
    Eigen3::Eigen
)
target_include_directories(robot
 PUBLIC
   "${CMAKE_CURRENT_SOURCE_DIR}/include"
)



add_executable(main main.cc)
target_link_libraries(main Eigen3::Eigen robot)
target_include_directories(main PUBLIC "${PROJECT_SOURCE_DIR}/inlcude")


# Tests
enable_testing()

add_executable(
  robot_test
  ${CMAKE_CURRENT_SOURCE_DIR}/src/robot_test.cc
)
target_link_libraries(robot_test
 PRIVATE
  GTest::gtest_main
  robot
)
target_include_directories(robot_test
 PUBLIC
   "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

include(GoogleTest)
gtest_discover_tests(robot_test)