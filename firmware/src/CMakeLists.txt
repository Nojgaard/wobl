file(GLOB_RECURSE SRCS "./*.cpp")

list(FILTER SRCS EXCLUDE REGEX ".*/calibrate/.*")

idf_component_register(
    SRCS ${SRCS}
    INCLUDE_DIRS "../include"
    REQUIRES "bluepad32-arduino" "scservo" "simplefoc" "icm20948")
