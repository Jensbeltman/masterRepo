cmake_minimum_required(VERSION 3.17)
project(master
        VERSION 1.0
        LANGUAGES CXX)

#PCL
find_package(PCL REQUIRED)
#JSON
find_package(nlohmann_json 3.2.0 REQUIRED)
#OPENCV
find_package(OpenCV REQUIRED)
# FCL
find_package(fcl REQUIRED)

add_subdirectory(lib)
add_subdirectory(app)
add_subdirectory(test)
