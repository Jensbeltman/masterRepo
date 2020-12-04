cmake_minimum_required(VERSION 3.12)
project(ContextProcessingTests
        VERSION 1.0
        LANGUAGES CXX)

add_executable(ViewPointCloudTest ViewPointCloudTest.cpp)
target_link_libraries(ViewPointCloudTest PUBLIC Dataset GA)
