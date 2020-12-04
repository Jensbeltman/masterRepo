cmake_minimum_required(VERSION 3.12)
project(Baseline
        VERSION 1.0
        LANGUAGES CXX)

add_library(Baseline SHARED)
target_sources(Baseline
        PRIVATE src/baseline.cpp)
target_include_directories(Baseline PUBLIC include)
target_link_libraries(Baseline PUBLIC GA)
target_compile_features(Baseline PUBLIC cxx_std_17)

