idf_component_register(
    SRCS         "src/led.cpp"
    INCLUDE_DIRS "include"
    REQUIRES     esp_event esp_driver_ledc esp_driver_gpio zigbee
)
