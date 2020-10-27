cmake_minimum_required(VERSION 3.12)
project(GA
        VERSION 1.0
        LANGUAGES CXX)

add_library(GA SHARED)
target_sources(GA
        PRIVATE
        src/typedefinitions.cpp
        src/ga.cpp
        src/GeneticEvaluator.cpp
        src/GeneticEvaluatorObjectCandidates.cpp
        src/collision_checking.cpp
        src/visualization.cpp
        )

target_include_directories(GA PUBLIC include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(GA PUBLIC ${OpenCV_LIBS} ${PCL_LIBRARIES} Chronometer Dataset fcl)