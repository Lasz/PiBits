cmake_minimum_required(3.12..3.16)
project(ledek
        LANGUAGES CXX
        VERSION 0.1
)
set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} -s -O3 -Wno-psabi)
add_executable()

