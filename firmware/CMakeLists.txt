cmake_minimum_required(VERSION 3.16)

# C++23 with GNU extensions across the whole project. gcc 13.2 (the version
# ESP-IDF v5.3 ships) supports nearly all of C++23. We don't enable -std=gnu++26
# yet — gcc 13 has only partial C++26 and we don't need anything from it.
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(hv-mrf-01)
