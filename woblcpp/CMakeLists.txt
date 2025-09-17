cmake_minimum_required(VERSION 3.10.0)
project(woblcpp VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Select the correct package depending on architecture
if(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
  set(ZENOHC_URL "https://github.com/eclipse-zenoh/zenoh-c/releases/download/1.5.1/zenoh-c-1.5.1-x86_64-unknown-linux-gnu-standalone.zip")
  set(ZENOHC_SHA256 "38d5802fc7a239b2dfb0170bdd837894ddc3c2cf26948869cb30d97abd71cf22")

  set(ZENOHCPP_URL "https://github.com/eclipse-zenoh/zenoh-cpp/releases/download/1.5.1/zenohcpp-1.5.1-standalone.zip")
  set(ZENOHCPP_SHA256 "c32f496a68975964df4f28a0e6d5c061448484424ace5f1a6ea149a8e461037c")
elseif(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
  set(ZENOHC_URL "https://github.com/eclipse-zenoh/zenoh-c/releases/download/1.5.1/zenoh-c-1.5.1-aarch64-unknown-linux-gnu-standalone.zip")
  set(ZENOHC_SHA256 "f47a51b6da35314c22aae84dddf224bd138ca21691b9f7d68314da0550275f0a")

  set(ZENOHCPP_URL "https://github.com/eclipse-zenoh/zenoh-cpp/releases/download/1.5.1/zenoh-cpp-1.5.1-aarch64-apple-darwin-standalone.zip")
  set(ZENOHCPP_SHA256 "c32f496a68975964df4f28a0e6d5c061448484424ace5f1a6ea149a8e461037c")
else()
  message(FATAL_ERROR "Unsupported architecture: ${CMAKE_SYSTEM_PROCESSOR}")
endif()

# External libraries
include(FetchContent)

# Fetch zenoh-c
FetchContent_Declare(
  zenohc
  URL ${ZENOHC_URL}
  URL_HASH SHA256=${ZENOHC_SHA256}
  DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)

# Fetch zenoh-cpp
FetchContent_Declare(
  zenohcpp
  URL ${ZENOHCPP_URL}
  URL_HASH SHA256=${ZENOHCPP_SHA256}
  DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)

FetchContent_MakeAvailable(zenohc zenohcpp)

# Add the downloaded cmake modules to the prefix path
list(APPEND CMAKE_PREFIX_PATH "${zenohc_SOURCE_DIR}/lib/cmake")
list(APPEND CMAKE_PREFIX_PATH "${zenohcpp_SOURCE_DIR}/lib/cmake")

# Include directories from downloaded packages
include_directories(${zenohc_SOURCE_DIR}/include ${zenohcpp_SOURCE_DIR}/include)


FetchContent_Declare(
  icm20948
  GIT_REPOSITORY https://github.com/makkhudjakov/SparkFun_ICM-20948_RaspberryLibrary.git
  #GIT_TAG main
  GIT_TAG 807f55b4b79a45bc064839fb67a0ffb3bb6e3377
)
FetchContent_MakeAvailable(icm20948)

find_package(zenohc REQUIRED)
find_package(zenohcxx REQUIRED)
find_package(Protobuf REQUIRED)

add_subdirectory(external/scservo)
include_directories(include external)

file(GLOB WOBL_MESSAGES_SRC "include/wobl/msg/*.cc")
file(GLOB_RECURSE WOBL_SRC "src/*.cpp")

add_library(woblcpp woblcpp.cpp ${WOBL_MESSAGES_SRC} ${WOBL_SRC})
target_link_libraries(woblcpp PUBLIC 
    ICM20948_Lib 
    SCServo
    protobuf::libprotobuf 
    zenohcxx::zenohc
)

add_subdirectory(scripts)
