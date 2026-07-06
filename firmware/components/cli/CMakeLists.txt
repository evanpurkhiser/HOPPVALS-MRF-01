idf_component_register(
    SRCS         "src/console.cpp"
    INCLUDE_DIRS "include"
    REQUIRES     console motor encoder motion current_sense self_test config reboot   # `console` here is ESP-IDF's esp_console component
)
