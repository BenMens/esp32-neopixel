idf_component_register(
    SRCS 
        "neopixel.cpp"
        "ws2812-encoder.cpp"
    REQUIRES 
        driver
    INCLUDE_DIRS "include"
)
