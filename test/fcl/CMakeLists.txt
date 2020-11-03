cmake_minimum_required(VERSION 3.12)
project(fcl_test
        VERSION 1.0
        LANGUAGES CXX)

add_executable(fcl_test fcl_test.cpp)
target_link_libraries(fcl_test PUBLIC GA)