cmake_minimum_required(VERSION 3.17)
project(plyreader)

add_library(plyreader src/tinyply.cpp)
target_include_directories(plyreader PUBLIC include)