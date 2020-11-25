cmake_minimum_required(VERSION 3.12)
project(ContextProcessing
        VERSION 1.0
        LANGUAGES CXX)

add_library(ContextProcessing)
target_sources(ContextProcessing
        PRIVATE
        src/ViewPointCloud.cpp
        )

target_include_directories(ContextProcessing PUBLIC include)
target_link_libraries(ContextProcessing PUBLIC ${PCL_LIBRARIES} Chronometer)