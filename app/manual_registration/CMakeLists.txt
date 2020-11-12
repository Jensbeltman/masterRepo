cmake_minimum_required(VERSION 3.2)
project(manual_registration
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

add_executable(manual_registration manual_registration.ui  manual_registration.cpp main.cpp)

target_include_directories(manual_registration PUBLIC ${CMAKE_CURRENT_BINARY_DIR} ${VTK_INCLUDE_DIRS})

target_link_libraries(manual_registration ${PCL_LIBRARIES}  ${VTK_LIBRARIES} Qt5::Widgets)

target_compile_features(manual_registration PUBLIC cxx_std_17)