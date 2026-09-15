#include "solar_exposure.h"
#include "persistent_settings.h"

namespace solar_exposure {

namespace {
struct LegacyProfileV2 {
  uint32_t magic;
  uint8_t version;
  uint8_t start_count;
  uint8_t end_count;
  uint8_t reserved;
  Observation starts[MAX_OBSERVATIONS];
  Observation ends[MAX_OBSERVATIONS];
};
static_assert(sizeof(LegacyProfileV2) == 1544, "Unexpected solar v2 profile size");
}

bool load() {
  Profile candidate{};
  if (blind_settings::load(blind_settings::K_SOLAR_EXPOSURE_PROFILE, candidate) &&
      candidate.magic == MAGIC && candidate.version == VERSION &&
      candidate.start_count <= MAX_OBSERVATIONS && candidate.limit_count <= MAX_OBSERVATIONS &&
      candidate.release_count <= MAX_OBSERVATIONS) {
    profile = candidate;
    loaded = true;
    return true;
  }

  blind_settings::FsFile file = blind_settings::fs_open(
      blind_settings::path_for(blind_settings::K_SOLAR_EXPOSURE_PROFILE), "r");
  LegacyProfileV2 legacy{};
  if (file && file.size() == sizeof(legacy) &&
      file.read(reinterpret_cast<uint8_t *>(&legacy), sizeof(legacy)) == sizeof(legacy)) {
    file.close();
    if (legacy.magic == MAGIC && legacy.version == 2 &&
        legacy.start_count <= MAX_OBSERVATIONS && legacy.end_count <= MAX_OBSERVATIONS) {
      reset();
      profile.start_count = legacy.start_count;
      profile.release_count = legacy.end_count;
      std::copy(legacy.starts, legacy.starts + legacy.start_count, profile.starts);
      std::copy(legacy.ends, legacy.ends + legacy.end_count, profile.releases);
      return save();
    }
  } else if (file) {
    file.close();
  }

  reset();
  return false;
}

bool save() {
  return blind_settings::save(blind_settings::K_SOLAR_EXPOSURE_PROFILE, profile);
}

}  // namespace solar_exposure
