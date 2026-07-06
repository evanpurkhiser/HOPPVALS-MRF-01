idf_component_register(
    SRCS         "src/motion.cpp"
    INCLUDE_DIRS "include"
    REQUIRES     motor encoder config zigbee esp_event
)
