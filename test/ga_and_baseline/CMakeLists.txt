cmake_minimum_required(VERSION 3.12)
project(ga_baseline_test
        VERSION 1.0
        LANGUAGES CXX)

add_executable(ga_scape_test ga_test_scape_data.cpp)
target_link_libraries(ga_scape_test PRIVATE GA)

add_executable(baseline baseline.cpp include/test_util.hpp ../ContextProcessingTests/ViewPointCloudTest.cpp)
target_link_libraries(baseline PUBLIC GA)

add_executable(baseline_ga_diff)
target_sources(baseline_ga_diff PRIVATE baseline_ga_diff.cpp)
target_include_directories(baseline_ga_diff PRIVATE include)
target_link_libraries(baseline_ga_diff PRIVATE GA)
