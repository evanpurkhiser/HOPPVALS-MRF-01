idf_component_register(
    SRCS        "src/zigbee.cpp"
    INCLUDE_DIRS "include"
    REQUIRES    nvs_flash esp_event
    PRIV_REQUIRES vfs
)
