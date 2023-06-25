idf_component_register(
        SRCS "keyboard.c"
        INCLUDE_DIRS include
        REQUIRES "i2c-pca9555" "i2c-ktd2052"
)
