idf_component_register(
    SRCS         "src/console.cpp"
    INCLUDE_DIRS "include"
    REQUIRES     console motor encoder motion current_sense self_test config esp_system   # `console` here is ESP-IDF's esp_console component
)
