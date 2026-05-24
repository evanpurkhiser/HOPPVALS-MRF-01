#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "esp_zigbee.h"

#include "blinds.h"

static const char *TAG = "BLINDS";

// Skeleton signal handler. Logs lifecycle events and drives commissioning.
// We'll wire the Window Covering cluster up here once the toolchain build is
// confirmed working end-to-end.
static bool blinds_signal_handler(const ezb_app_signal_t *app_signal)
{
    ezb_app_signal_type_t signal_type = ezb_app_signal_get_type(app_signal);

    switch (signal_type) {
    case EZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "Initialize Zigbee stack");
        ezb_bdb_start_top_level_commissioning(EZB_BDB_MODE_INITIALIZATION);
        break;
    case EZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case EZB_BDB_SIGNAL_DEVICE_REBOOT:
        ESP_LOGI(TAG, "Zigbee device started%s", ezb_bdb_is_factory_new() ? " (factory new)" : "");
        if (ezb_bdb_is_factory_new()) {
            ezb_bdb_start_top_level_commissioning(EZB_BDB_MODE_NETWORK_STEERING);
        }
        break;
    case EZB_BDB_SIGNAL_STEERING: {
        ezb_bdb_comm_status_t status = *((ezb_bdb_comm_status_t *)ezb_app_signal_get_params(app_signal));
        if (status == EZB_BDB_STATUS_SUCCESS) {
            ESP_LOGI(TAG, "Joined network (PAN 0x%04hx, ch %d)",
                     ezb_nwk_get_panid(), ezb_nwk_get_current_channel());
        } else {
            ESP_LOGW(TAG, "Network steering failed (0x%02x), retrying...", status);
        }
    } break;
    default:
        ESP_LOGI(TAG, "Zigbee signal: %s (0x%02x)",
                 ezb_app_signal_to_string(signal_type), signal_type);
        break;
    }
    return true;
}

// TODO: replace this stub with a real Window Covering endpoint:
//   - Basic cluster (manufacturer, model)
//   - Identify cluster
//   - Groups / Scenes
//   - Window Covering cluster (0x0102) with attrs: WindowCoveringType,
//     CurrentPositionLiftPercentage, Mode, ConfigStatus; commands:
//     UpOrOpen, DownOrClose, Stop, GoToLiftPercentage.
static esp_err_t blinds_register_endpoint(void)
{
    ezb_af_device_desc_t dev_desc = ezb_af_create_device_desc();
    ESP_ERROR_CHECK(ezb_af_device_desc_register(dev_desc));
    return ESP_OK;
}

static esp_err_t blinds_setup_commissioning(void)
{
    ezb_aps_secur_enable_distributed_security(false);
    ESP_ERROR_CHECK(ezb_bdb_set_primary_channel_set(ESP_ZIGBEE_PRIMARY_CHANNEL_MASK));
    ESP_ERROR_CHECK(ezb_bdb_set_secondary_channel_set(ESP_ZIGBEE_SECONDARY_CHANNEL_MASK));
    ESP_ERROR_CHECK(ezb_app_signal_add_handler(blinds_signal_handler));
    return ESP_OK;
}

static void blinds_zigbee_task(void *pvParameters)
{
    esp_zigbee_config_t config = BLINDS_DEFAULT_CONFIG();

    ESP_ERROR_CHECK(esp_zigbee_init(&config));
    ESP_ERROR_CHECK(blinds_setup_commissioning());
    ESP_ERROR_CHECK(blinds_register_endpoint());
    ESP_ERROR_CHECK(esp_zigbee_start(false));

    esp_zigbee_launch_mainloop();

    esp_zigbee_deinit();
    vTaskDelete(NULL);
}

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(nvs_flash_init_partition(ESP_ZIGBEE_STORAGE_PARTITION_NAME));
    ESP_LOGI(TAG, "Starting blinds firmware");
    xTaskCreate(blinds_zigbee_task, "zb_main", 4096, NULL, 5, NULL);
}
