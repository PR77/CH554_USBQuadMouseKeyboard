from os import path
Import("env")

env.AddPostAction(
    "$BUILD_DIR/${PROGNAME}.hex",
    env.VerboseAction(" ".join([
        "$OBJCOPY",
        "-I",
        "ihex",
        "-O",
        "binary",
        "$BUILD_DIR/${PROGNAME}.hex",
        "$BUILD_DIR/${PROGNAME}.bin",
    ]), "Creating binary $BUILD_DIR/${PROGNAME}.bin")
)