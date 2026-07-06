file(GLOB_RECURSE SRCS
    "${WOBL_LIBDEPS}/SparkFun 9DoF IMU Breakout - ICM 20948 - Arduino Library/src/*.cpp"
    "${WOBL_LIBDEPS}/SparkFun 9DoF IMU Breakout - ICM 20948 - Arduino Library/src/*.c"
)

idf_component_register(
    SRCS ${SRCS}
    INCLUDE_DIRS "${WOBL_LIBDEPS}/SparkFun 9DoF IMU Breakout - ICM 20948 - Arduino Library/src"
    REQUIRES arduino-esp32)

target_compile_definitions(${COMPONENT_LIB} PRIVATE ICM_20948_USE_DMP)
