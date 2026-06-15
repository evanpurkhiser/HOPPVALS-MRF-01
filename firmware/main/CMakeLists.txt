idf_component_register(
    SRCS         "main.cpp"
    INCLUDE_DIRS "."
    REQUIRES     zigbee led motor encoder motion current_sense cli esp_event
)
