// Top-level wiring. main.cpp is intentionally thin: it brings the default
// event loop up, registers an application-level listener for Zigbee events,
// and hands off to the zigbee component which owns the actual stack.
//
// Once we add motor/motion/persistence components, they'll each register
// handlers on the same event bus. This file shouldn't grow.

#include <cinttypes>

#include "esp_event.h"
#include "esp_log.h"

#include "blinds/console.hpp"
#include "blinds/current_sense.hpp"
#include "blinds/encoder.hpp"
#include "blinds/led.hpp"
#include "blinds/motion.hpp"
#include "blinds/motor.hpp"
#include "blinds/zigbee.hpp"

namespace {

constexpr auto *TAG = "blinds.app";

void on_zigbee_event(void *, esp_event_base_t, std::int32_t id, void *data)
{
    using blinds::zigbee::Event;
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
    ESP_ERROR_CHECK(esp_event_handler_register(blinds::zigbee::EVENTS,
                                               ESP_EVENT_ANY_ID, &on_zigbee_event,
                                               nullptr));
    ESP_LOGI(TAG, "Starting blinds firmware");
    blinds::motor::start();          // registers cover handlers, enables drivers
    blinds::encoder::start();        // PCNT quadrature readers for both motors
    blinds::current_sense::start();  // ADC1 IPROPI sampling task (100 Hz)
    blinds::motion::start();         // 100 Hz closed-loop speed control task
    blinds::led::start();
    blinds::console::start();        // serial REPL — needs motor + encoder up first
    blinds::zigbee::start();
}
