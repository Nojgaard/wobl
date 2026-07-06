file(GLOB_RECURSE SRCS
    "${WOBL_LIBDEPS}/Simple FOC/src/*.cpp"
    "${WOBL_LIBDEPS}/Simple FOC/src/*.c"
)

idf_component_register(
    SRCS ${SRCS}
    INCLUDE_DIRS "${WOBL_LIBDEPS}/Simple FOC/src"
    REQUIRES arduino-esp32)