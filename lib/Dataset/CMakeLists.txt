cmake_minimum_required(VERSION 3.12)
project(Dataset
        VERSION 1.0
        LANGUAGES CXX)

add_library(Dataset src/DataPoint.cpp include/dataset/DataPoint.hpp)
target_sources(Dataset
        PRIVATE src/scape/ScapeDataset.cpp
                src/scape/ScapeDatasetObject.cpp
        src/scape/ScapeDataPoint.cpp
                src/Dataset.cpp
                src/DatasetObject.cpp
                src/sileane/SileaneDataset.cpp
                src/sileane/SileaneDatasetObject.cpp
                src/sileane/SileaneCameraParams.cpp
        src/transform_utility.cpp)
target_include_directories(Dataset PUBLIC include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(Dataset PUBLIC ${PCL_LIBRARIES} opencv_core opencv_viz opencv_imgcodecs)
target_compile_features(Dataset PUBLIC cxx_std_20)

