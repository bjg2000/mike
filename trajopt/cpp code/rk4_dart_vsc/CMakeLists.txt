cmake_minimum_required(VERSION 3.10.0)
project(rk4_dart_cmake VERSION 0.1.0 LANGUAGES C)

add_executable(rk4_dart_cmake main.c)

include(CTest)
enable_testing()

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
