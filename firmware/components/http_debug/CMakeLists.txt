idf_component_register(
    SRCS         "src/http_debug.cpp"
    INCLUDE_DIRS "include"
    REQUIRES     esp_wifi esp_netif esp_event esp_http_server app_update config cli motor motion utils logtap
)
