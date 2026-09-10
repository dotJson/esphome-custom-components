import esphome.codegen as cg
import esphome.config_validation as cv

CODEOWNERS = ["@dotJson"]

CONFIG_SCHEMA = cv.Schema({})


async def to_code(config):
    cg.add_global(cg.RawStatement('#include "esphome/components/blinds_common/persistent_settings.h"'))
    cg.add_global(cg.RawStatement('#include "esphome/components/blinds_common/activity_events.h"'))
    cg.add_global(cg.RawStatement('#include "esphome/components/blinds_common/webhook_transport.h"'))
    cg.add_global(cg.RawStatement('#include "esphome/components/blinds_common/storage_diagnostics.h"'))
