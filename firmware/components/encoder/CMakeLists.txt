idf_component_register(
    SRCS         "src/encoder.cpp"
    INCLUDE_DIRS "include"
    REQUIRES     esp_driver_gpio esp_driver_pcnt motor
)
