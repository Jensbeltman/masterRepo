cmake_minimum_required(VERSION 3.12)
project(Chronometer
        VERSION 1.0
        LANGUAGES CXX)

add_library(Chronometer INTERFACE)
target_include_directories(Chronometer INTERFACE include)
