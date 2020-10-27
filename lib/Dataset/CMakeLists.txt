cmake_minimum_required(VERSION 3.12)
project(Dataset
        VERSION 1.0
        LANGUAGES CXX)

add_library(Dataset INTERFACE)
target_include_directories(Dataset INTERFACE include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(Dataset INTERFACE ${PCL_LIBRARIES} opencv_core opencv_viz opencv_imgcodecs)
target_compile_features(Dataset INTERFACE cxx_std_17)

