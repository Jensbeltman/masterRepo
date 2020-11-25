cmake_minimum_required(VERSION 3.2)
project(viewpoint_point_cloud
        VERSION 1.0
        LANGUAGES CXX)


find_package(VTK REQUIRED) # Needed for include dirs to be found dont know why it cant be propergates from the top level cmake
include(${VTK_USE_FILE})

add_executable(viewpoint_point_cloud viewpoint_point_cloud.cpp)

target_include_directories(viewpoint_point_cloud PUBLIC ${VTK_INCLUDE_DIRS})

target_link_libraries(viewpoint_point_cloud PUBLIC ${PCL_LIBRARIES}  ${VTK_LIBRARIES} GA)

target_compile_features(viewpoint_point_cloud PUBLIC cxx_std_17) # std filesystem is needed