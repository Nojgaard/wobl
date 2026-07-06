cmake_minimum_required(VERSION 3.16.0)

set(WOBL_LIBDEPS "${CMAKE_CURRENT_LIST_DIR}/.pio/libdeps/release")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(wobl-esp32)
