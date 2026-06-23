idf_component_register(
    SRCS         "src/motor.cpp"
    INCLUDE_DIRS "include"
    REQUIRES     esp_driver_gpio esp_driver_ledc types
)
