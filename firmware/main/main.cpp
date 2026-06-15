// Top-level wiring. main.cpp is intentionally thin: it brings the default
// event loop up, registers an application-level listener for Zigbee events,
// and hands off to the zigbee component which owns the actual stack.
//
// Once we add motor/motion/persistence components, they'll each register
// handlers on the same event bus. This file shouldn't grow.

#include <cinttypes>

#include "esp_event.h"
#include "esp_log.h"

#include "hv-mrf-01/console.hpp"
#include "hv-mrf-01/current_sense.hpp"
#include "hv-mrf-01/encoder.hpp"
#include "hv-mrf-01/led.hpp"
#include "hv-mrf-01/motion.hpp"
#include "hv-mrf-01/motor.hpp"
#include "hv-mrf-01/zigbee.hpp"

namespace {

constexpr auto *TAG = "hv-mrf-01.app";

void on_zigbee_event(void *, esp_event_base_t, std::int32_t id, void *data)
{
    using hvmrf01::zigbee::Event;
    switch (static_cast<Event>(id)) {
    case Event::JoinedNetwork:
        ESP_LOGI(TAG, "→ joined zigbee network");
        break;
    case Event::LeftNetwork:
        ESP_LOGI(TAG, "→ left zigbee network");
        break;
    case Event::Identify: {
        const auto effect = *static_cast<std::uint8_t *>(data);
        ESP_LOGI(TAG, "→ identify effect 0x%02x", effect);
    } break;
    }
}


}  // namespace

extern "C" void app_main()
{
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(esp_event_handler_register(hvmrf01::zigbee::EVENTS,
                                               ESP_EVENT_ANY_ID, &on_zigbee_event,
                                               nullptr));
    ESP_LOGI(TAG, "Starting hv-mrf-01 firmware");
    hvmrf01::motor::start();          // registers cover handlers, enables drivers
    hvmrf01::encoder::start();        // PCNT quadrature readers for both motors
    hvmrf01::current_sense::start();  // ADC1 IPROPI sampling task (100 Hz)
    hvmrf01::motion::start();         // 100 Hz closed-loop speed control task
    hvmrf01::led::start();
    hvmrf01::console::start();        // serial REPL — needs motor + encoder up first
    hvmrf01::zigbee::start();
}
