cmake_minimum_required(VERSION 3.2)
project(dataset_viewer
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

add_executable(dataset_viewer)

target_sources(dataset_viewer PRIVATE dataset_viewer.ui manual_registration.ui manual_registration.cpp main.cpp dataset_viewer.cpp)

target_include_directories(dataset_viewer PUBLIC ./ ../manual_registration ${CMAKE_CURRENT_BINARY_DIR} ${VTK_INCLUDE_DIRS})

target_link_libraries(dataset_viewer PUBLIC ${PCL_LIBRARIES}  ${VTK_LIBRARIES} Qt5::Widgets Dataset HV)

target_compile_features(dataset_viewer PUBLIC cxx_std_17)