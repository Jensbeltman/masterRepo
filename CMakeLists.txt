cmake_minimum_required(VERSION 3.17)
project(master
        VERSION 1.0
        LANGUAGES CXX)


find_package(Qt5 COMPONENTS Widgets REQUIRED)
find_package(VTK REQUIRED)
OPTION(VTK_LEGACY_REMOVE "Remove all legacy code completely." ON)
#PCL
find_package(PCL 1.10 REQUIRED)
#JSON
find_package(nlohmann_json 3.2.0 REQUIRED)
# FCL
find_package(fcl REQUIRED)
#OPENCV
find_package(OpenCV REQUIRED)
#Matplot++
find_package(Matplot++ REQUIRED)

include(FindPkgConfig)
pkg_check_modules(libnlopt REQUIRED)
pkg_check_modules(bayesopt REQUIRED)



add_subdirectory(lib)
add_subdirectory(app)
add_subdirectory(test)
