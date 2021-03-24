cmake_minimum_required(VERSION 3.12)
project(bayesOptTuning
        VERSION 1.0
        LANGUAGES CXX)

add_executable(bo_tuning bo_tuning.cpp)
target_link_libraries(bo_tuning PUBLIC bayesopt HV Dataset Chronometer)


