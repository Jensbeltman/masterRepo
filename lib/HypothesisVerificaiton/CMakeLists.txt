cmake_minimum_required(VERSION 3.12)
project(HV
        VERSION 1.0
        LANGUAGES CXX)

add_library(HV SHARED)

target_sources(HV
        PRIVATE
        src/typedefinitions.cpp
        src/hv_alg/bf.cpp
        src/hv_alg/bf_gen.cpp
        src/hv_alg/ga.cpp
        src/hv_alg/sequential_prior.cpp
        src/hv_alg/ga_functions.cpp
        src/visualization/point_cloud_group_visualizer.cpp
        src/evaluator/GeneticEvaluator.cpp
        src/evaluator/GeneticEvaluatorInlierCollision.cpp
        src/evaluator/GeneticEvaluatorInlierCollisionVariants.cpp
        src/evaluator/functionality/VisibleInlier.cpp
        src/evaluator/functionality/Collision.cpp
        src/evaluator/functionality/IntersectingPoints.cpp
        src/evaluator/collision_checking.cpp
        src/evaluator/simple_visibility_function.cpp
        src/evaluator/point_cloud_renderer.cpp
        )

target_include_directories(HV PUBLIC include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(HV PUBLIC ${OpenCV_LIBS} ${PCL_LIBRARIES} Chronometer Dataset fcl)
target_compile_features(HV PUBLIC cxx_std_17)