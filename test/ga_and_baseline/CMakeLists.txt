cmake_minimum_required(VERSION 3.12)
project(ga_baseline_test
        VERSION 1.0
        LANGUAGES CXX)

add_executable(ga_scape_test ga_test_scape_data.cpp)
target_link_libraries(ga_scape_test PRIVATE GA)

add_executable(comp comp.cpp include/test_util.hpp)
target_link_libraries(comp PUBLIC Chronometer DataUtil Matplot++::matplot)

add_executable(optimization_algorithm_test.cpp optimization_algorithm_test.cpp)
target_link_libraries(optimization_algorithm_test.cpp PUBLIC Chronometer Dataset DataUtil HV)

add_executable(baseline_ga_diff)
target_sources(baseline_ga_diff PRIVATE baseline_ga_diff.cpp)
target_include_directories(baseline_ga_diff PRIVATE include)
target_link_libraries(baseline_ga_diff PRIVATE GA Baseline DataUtil)
