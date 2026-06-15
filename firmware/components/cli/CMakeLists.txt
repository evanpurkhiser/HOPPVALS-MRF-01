idf_component_register(
    SRCS         "src/console.cpp"
    INCLUDE_DIRS "include"
    REQUIRES     console motor encoder motion current_sense   # `console` here is ESP-IDF's esp_console component
)
