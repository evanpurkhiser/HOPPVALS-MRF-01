idf_component_register(
    SRCS         "src/reboot.cpp"
    INCLUDE_DIRS "include"
    REQUIRES     config motion motor esp_system esp_timer
)
