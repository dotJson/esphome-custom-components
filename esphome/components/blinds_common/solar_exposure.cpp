#include "solar_exposure.h"
#include "persistent_settings.h"

namespace solar_exposure {

bool load() {
  Profile candidate{};
  if (!blind_settings::load(blind_settings::K_SOLAR_EXPOSURE_PROFILE, candidate) ||
      candidate.magic != MAGIC || candidate.version != VERSION ||
      candidate.start_count > MAX_OBSERVATIONS || candidate.end_count > MAX_OBSERVATIONS) {
    reset();
    return false;
  }
  profile = candidate;
  loaded = true;
  return true;
}

bool save() {
  return blind_settings::save(blind_settings::K_SOLAR_EXPOSURE_PROFILE, profile);
}

}  // namespace solar_exposure
