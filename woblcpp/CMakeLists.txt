cmake_minimum_required(VERSION 3.10.0)
project(woblcpp VERSION 0.1.0 LANGUAGES C CXX)

# External libraries
include(FetchContent)
FetchContent_Declare(
  icm20948
  GIT_REPOSITORY https://github.com/makkhudjakov/SparkFun_ICM-20948_RaspberryLibrary.git
  #GIT_TAG main
  GIT_TAG 807f55b4b79a45bc064839fb67a0ffb3bb6e3377
)
FetchContent_MakeAvailable(icm20948)

add_library(woblcpp woblcpp.cpp)

