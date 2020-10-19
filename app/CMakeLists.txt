cmake_minimum_required(VERSION 3.12)
project(app
        VERSION 1.0
        LANGUAGES CXX)


add_executable(main src/main.cpp)
add_definitions(${PCL_DEFINITIONS})
target_include_directories(main PUBLIC ${OpenCV_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${fcl_INCLUDE_DIRS})
target_link_libraries(main PRIVATE ${PCL_LIBRARIES} ${OPENCV_LIBS} nlohmann_json::nlohmann_json fcl Chronometer Dataset GA)