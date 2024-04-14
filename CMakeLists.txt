cmake_minimum_required(VERSION 3.14)
set(CMAKE_FIND_DEBUG_MODE 0)
project(alaurens VERSION 1.0)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

find_package(Eigen3 3.3 REQUIRED)
find_package(GTest CONFIG REQUIRED)
find_package(absl REQUIRED)


# utils
add_library(robotics_lib_utils INTERFACE)
target_compile_features(robotics_lib_utils INTERFACE cxx_std_14)
target_sources(robotics_lib_utils PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include/utils.h")

# Robot library
set(SRCS
  "${CMAKE_CURRENT_SOURCE_DIR}/src/robot.cc"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/transformations.cc"
)
set(HDRS
  "${CMAKE_CURRENT_SOURCE_DIR}/include/robot.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/include/transformations.h"
)
add_library(robotics_lib)
target_sources(robotics_lib PUBLIC ${HDRS} PRIVATE ${SRCS})

target_link_libraries(robotics_lib
  PUBLIC
    absl::strings
    absl::span
    absl::algorithm
  PRIVATE
    Eigen3::Eigen
    robotics_lib_utils
)
target_include_directories(robotics_lib
 PUBLIC
   "${CMAKE_CURRENT_SOURCE_DIR}/include"
)



add_executable(main main.cc)
target_link_libraries(main Eigen3::Eigen robotics_lib)
target_include_directories(main PUBLIC "${PROJECT_SOURCE_DIR}/inlcude")


# Tests
include(GoogleTest)
enable_testing()

macro(AddTest)
    set(options)
    set(one_value_args NAME SOURCE)
    set(multi_value_args)
    cmake_parse_arguments(_ARGS
      "${options}"
      "${one_value_args}"
      "${multi_value_args}"
      ${ARGN}
    )

    add_executable(${_ARGS_NAME} ${_ARGS_SOURCE})
    target_link_libraries(${_ARGS_NAME}
      GTest::gtest_main
      GTest::gmock
      GTest::gtest
      robotics_lib
      Eigen3::Eigen
    )
    gtest_discover_tests(${_ARGS_NAME})
endmacro()


AddTest(
  NAME robot_test
  SOURCE  "${CMAKE_CURRENT_SOURCE_DIR}/src/robot_test.cc"
)

AddTest(
  NAME transformations_test
  SOURCE  "${CMAKE_CURRENT_SOURCE_DIR}/src/transformations_test.cc"
)