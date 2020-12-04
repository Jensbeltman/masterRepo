cmake_minimum_required(VERSION 3.12)
project(GA
        VERSION 1.0
        LANGUAGES CXX)

add_library(GA SHARED)
target_sources(GA
        PRIVATE
        src/typedefinitions.cpp
        src/ga.cpp
        src/genetic_evalutator/GeneticEvaluator.cpp
        src/genetic_evalutator/GeneticEvaluatorObjectCandidates.cpp
        src/collision/collision_checking.cpp
        src/visualization/visualization.cpp
        src/point_cloud_processing/point_cloud_processing.cpp
        src/visualization/point_cloud_renderer.cpp
        )

target_include_directories(GA PUBLIC include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(GA PUBLIC ${OpenCV_LIBS} ${PCL_LIBRARIES} Chronometer Dataset fcl)