idf_component_register(
    SRCS         "main.cpp"
    INCLUDE_DIRS "."
    REQUIRES     config zigbee led motor encoder motion current_sense cli http_debug esp_event esp_timer
)
