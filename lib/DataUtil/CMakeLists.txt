cmake_minimum_required(VERSION 3.12)
project(DataUtil
        VERSION 1.0
        LANGUAGES CXX)

add_library(DataUtil INTERFACE)
target_include_directories(DataUtil INTERFACE include)
target_link_libraries(DataUtil INTERFACE HV)
target_compile_features(DataUtil INTERFACE cxx_std_17)