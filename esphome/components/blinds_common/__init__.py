import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components.esp32 import include_builtin_idf_component

CODEOWNERS = ["@dotJson"]
DEPENDENCIES = ["esp32"]

CONFIG_SCHEMA = cv.Schema({})


async def to_code(config):
    # ESPHome 2026.2+ excludes the ESP-IDF HTTP client unless a component
    # explicitly requests it. webhook_transport.cpp uses esp_http_client.h.
    include_builtin_idf_component("esp_http_client")

    # Expose the component's public interfaces to generated YAML lambdas.
    cg.add_global(cg.RawStatement('#include "esphome/components/blinds_common/persistent_settings.h"'))
    cg.add_global(cg.RawStatement('#include "esphome/components/blinds_common/activity_events.h"'))
    cg.add_global(cg.RawStatement('#include "esphome/components/blinds_common/webhook_transport.h"'))
    cg.add_global(cg.RawStatement('#include "esphome/components/blinds_common/storage_diagnostics.h"'))
