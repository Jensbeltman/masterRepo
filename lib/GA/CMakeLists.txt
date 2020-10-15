cmake_minimum_required(VERSION 3.12)
project(GA
        VERSION 1.0
        LANGUAGES CXX)

add_library(GA SHARED include/ga/utility/logging.hpp)
target_sources(GA
        PRIVATE
        src/typedefinitions.cpp
        src/ga.cpp
        src/GeneticEvaluator.cpp
        src/GeneticEvaluatorObjectCandidates.cpp
        )

target_include_directories(GA PUBLIC include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(GA PUBLIC ${OpenCV_LIBS} ${PCL_LIBRARIES} fcl Chronometer Dataset)


