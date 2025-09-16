cmake_minimum_required(VERSION 3.10.0)
project(woblcpp VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# External libraries
include(FetchContent)

#FetchContent_Declare(
#  icm20948
#  GIT_REPOSITORY https://github.com/makkhudjakov/SparkFun_ICM-20948_RaspberryLibrary.git
#  #GIT_TAG main
#  GIT_TAG 807f55b4b79a45bc064839fb67a0ffb3bb6e3377
#)
#FetchContent_MakeAvailable(icm20948)

find_package(Protobuf CONFIG REQUIRED)
add_library(woblcpp woblcpp.cpp)
target_link_libraries(woblcpp PRIVATE protobuf::libprotobuf)
