idf_component_register(
    SRCS         "src/current_sense.cpp"
    INCLUDE_DIRS "include"
    REQUIRES     esp_adc esp_event motor
)
