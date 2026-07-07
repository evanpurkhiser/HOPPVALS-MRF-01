idf_component_register(
    SRCS        "src/zigbee.cpp"
    INCLUDE_DIRS "include"
    REQUIRES    nvs_flash esp_event esp_app_format event_log
    PRIV_REQUIRES vfs config
)
