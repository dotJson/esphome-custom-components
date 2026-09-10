#pragma once

// HTTP diagnostics endpoint for inspecting persistent settings stored in LittleFS.
//
// Keep this public interface available regardless of include order. The
// implementation itself is compiled only when ESP32 + web_server support is
// enabled; guarding these declarations on USE_WEBSERVER made the type alias
// disappear when this header was parsed before ESPHome's web-server headers.

namespace littlefs_web_dump {

using MotionQuery = bool (*)();

// Supplies the authoritative motor-motion state so diagnostics defer filesystem
// work and response streaming while the blind motor is moving.
void set_motion_query(MotionQuery query);

// Registers the /littlefs diagnostics endpoint with ESPHome's web server.
bool register_handler();

}  // namespace littlefs_web_dump
