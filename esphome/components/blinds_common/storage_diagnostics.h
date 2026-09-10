#pragma once

// HTTP diagnostics endpoint for inspecting persistent settings stored in LittleFS.

#if defined(USE_ESP32) && defined(USE_WEBSERVER)

namespace littlefs_web_dump {

using MotionQuery = bool (*)();

// Supplies the authoritative motor-motion state so diagnostics defer filesystem
// work and response streaming while the blind motor is moving.
void set_motion_query(MotionQuery query);

// Registers the /littlefs diagnostics endpoint with ESPHome's web server.
bool register_handler();

}  // namespace littlefs_web_dump

#endif  // USE_ESP32 && USE_WEBSERVER
