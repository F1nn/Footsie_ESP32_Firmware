file(GLOB_RECURSE app_sources ${CMAKE_SOURCE_DIR}/src/*.c)
idf_component_register(
    SRCS
        ${app_sources}
    INCLUDE_DIRS
        "."
        "../include"
    REQUIRES
        esp_driver_gpio
        esp_driver_i2c
        esp_driver_spi
        esp_timer
        esp_event
)

