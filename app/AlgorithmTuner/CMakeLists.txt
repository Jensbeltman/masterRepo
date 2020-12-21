cmake_minimum_required(VERSION 3.2)
project(alogrithm_tuner
        VERSION 1.0
        LANGUAGES CXX)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

if(CMAKE_VERSION VERSION_LESS "3.7.0")
    set(CMAKE_INCLUDE_CURRENT_DIR ON)
endif()

find_package(VTK REQUIRED) # Needed for include dirs to be found dont know why it cant be propergates from the top level cmake
include(${VTK_USE_FILE})

add_executable(alogrithm_tuner)

target_sources(alogrithm_tuner PRIVATE algorithm_tuner.ui main.cpp algorithm_tuner.cpp)

target_include_directories(alogrithm_tuner PUBLIC ${CMAKE_CURRENT_BINARY_DIR} ${VTK_INCLUDE_DIRS})

target_link_libraries(alogrithm_tuner PUBLIC Chronometer ${PCL_LIBRARIES}  ${VTK_LIBRARIES} Qt5::Widgets Dataset GA Baseline)

target_compile_features(alogrithm_tuner PUBLIC cxx_std_17)