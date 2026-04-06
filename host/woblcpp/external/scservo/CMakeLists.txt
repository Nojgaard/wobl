file(GLOB hdrs *.h)
file(GLOB srs *.cpp)

add_library(SCServo STATIC ${hdrs} ${srs})