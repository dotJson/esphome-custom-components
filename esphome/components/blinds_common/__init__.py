from pathlib import Path
import json

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components.esp32 import add_extra_build_file, include_builtin_idf_component


CODEOWNERS = ["@dotJson"]
DEPENDENCIES = ["esp32"]

CONFIG_SCHEMA = cv.Schema({})

PARTITION_CSV = "partitions_esp32s3_n8_flash_map.csv"
TIMEZONE_JSON = "timezones_iana_to_posix.json"


def _load_timezone_map():
    source = Path(__file__).with_name(TIMEZONE_JSON)

    try:
        with source.open("r", encoding="utf-8") as file:
            data = json.load(file)
    except FileNotFoundError as err:
        raise cv.Invalid(
            f"blinds_common requires {TIMEZONE_JSON} beside __init__.py"
        ) from err
    except (OSError, json.JSONDecodeError) as err:
        raise cv.Invalid(f"Could not read {TIMEZONE_JSON}: {err}") from err

    if not isinstance(data, dict) or not data:
        raise cv.Invalid(f"{TIMEZONE_JSON} must contain a non-empty JSON object")

    entries = []
    for iana, posix in data.items():
        if not isinstance(iana, str) or not iana:
            raise cv.Invalid(f"{TIMEZONE_JSON} contains an invalid IANA name")
        if not isinstance(posix, str) or not posix:
            raise cv.Invalid(f"{TIMEZONE_JSON} contains an invalid POSIX value for {iana}")
        entries.append((iana, posix))

    entries.sort(key=lambda item: item[0])
    return entries


def _cpp_string(value):
    # JSON string escaping is valid for these ASCII C++ string literals.
    return json.dumps(value, ensure_ascii=True)


def _timezone_lookup_cpp(entries):
    rows = ",\n".join(
        f"  {{{_cpp_string(iana)}, {_cpp_string(posix)}}}" for iana, posix in entries
    )

    return f'''#include <cstring>

namespace blinds_common_timezone {{

struct TimezoneEntry {{
  const char *iana;
  const char *posix;
}};

static const TimezoneEntry TIMEZONE_ENTRIES[] = {{
{rows}
}};

static constexpr size_t TIMEZONE_COUNT =
    sizeof(TIMEZONE_ENTRIES) / sizeof(TIMEZONE_ENTRIES[0]);

inline const char *iana_to_posix(const char *iana) {{
  if (iana == nullptr || *iana == '\\0') return nullptr;

  size_t first = 0;
  size_t last = TIMEZONE_COUNT;

  while (first < last) {{
    const size_t middle = first + ((last - first) / 2U);
    const int comparison = std::strcmp(iana, TIMEZONE_ENTRIES[middle].iana);

    if (comparison == 0) return TIMEZONE_ENTRIES[middle].posix;
    if (comparison < 0) last = middle;
    else first = middle + 1U;
  }}

  return nullptr;
}}

inline const char *posix_to_iana(const char *posix, const char *preferred_iana = nullptr) {{
  if (posix == nullptr || *posix == '\\0') return nullptr;

  if (preferred_iana != nullptr) {{
    const char *preferred_posix = iana_to_posix(preferred_iana);
    if (preferred_posix != nullptr && std::strcmp(posix, preferred_posix) == 0) {{
      return preferred_iana;
    }}
  }}

  for (size_t index = 0; index < TIMEZONE_COUNT; index++) {{
    if (std::strcmp(posix, TIMEZONE_ENTRIES[index].posix) == 0) {{
      return TIMEZONE_ENTRIES[index].iana;
    }}
  }}

  return nullptr;
}}

}}  // namespace blinds_common_timezone
'''


async def to_code(config):
    # ESPHome 2026.2+ excludes the ESP-IDF HTTP client unless a component
    # explicitly requests it. webhook_transport.cpp uses esp_http_client.h.
    include_builtin_idf_component("esp_http_client")

    # Supply the custom ESP32-S3 N8 partition map from this external component.
    partition_source = Path(__file__).with_name(PARTITION_CSV)
    add_extra_build_file("partitions.csv", partition_source)
    cg.add_platformio_option("board_build.partitions", "partitions.csv")

    # Generate a flash-resident, alphabetically sorted IANA -> POSIX lookup table
    # from the JSON snapshot stored beside this file. The device never parses
    # JSON and never needs network access to change its runtime timezone.
    timezone_entries = _load_timezone_map()
    cg.add_global(cg.RawStatement(_timezone_lookup_cpp(timezone_entries)))

    # Expose the component's public interfaces to generated YAML lambdas.
    cg.add_global(cg.RawStatement('#include "esphome/components/blinds_common/persistent_settings.h"'))
    cg.add_global(cg.RawStatement('#include "esphome/components/blinds_common/activity_events.h"'))
    cg.add_global(cg.RawStatement('#include "esphome/components/blinds_common/webhook_transport.h"'))
    cg.add_global(cg.RawStatement('#include "esphome/components/blinds_common/storage_diagnostics.h"'))
