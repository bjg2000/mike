cmake_minimum_required(VERSION 3.10.0)
project(nlopt_test_cmake VERSION 0.1.0 LANGUAGES C)

find_package(nlopt REQUIRED)

add_executable(nlopt_test_cmake nlopt_test.c)

include(CTest)
enable_testing()

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
