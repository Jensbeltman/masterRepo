cmake_minimum_required(VERSION 3.12)
project(dataset_tests
        VERSION 1.0
        LANGUAGES CXX)

add_executable(sileane_test sileane_data_set_test.cpp)
target_link_libraries(sileane_test PUBLIC Chronometer Dataset)

add_executable(scape_test scape_data_set_test.cpp)
target_link_libraries(scape_test PUBLIC Chronometer Dataset)

add_executable(gt_collision_test gt_collision_test.cpp)
target_link_libraries(gt_collision_test PUBLIC Chronometer Dataset DataUtil HV)
