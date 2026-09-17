#include "solar_exposure.h"
#include "persistent_settings.h"

#include <esp_heap_caps.h>

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

template<typename T> T *allocate_profile_buffer() {
  void *buffer = heap_caps_calloc(1, sizeof(T), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (buffer == nullptr) buffer = heap_caps_calloc(1, sizeof(T), MALLOC_CAP_8BIT);
  return static_cast<T *>(buffer);
}
}

bool load() {
  Profile *candidate = allocate_profile_buffer<Profile>();
  if (candidate != nullptr) {
    const bool valid =
        blind_settings::load(blind_settings::K_SOLAR_EXPOSURE_PROFILE, *candidate) &&
        candidate->magic == MAGIC && candidate->version == VERSION &&
        candidate->start_count <= MAX_OBSERVATIONS && candidate->limit_count <= MAX_OBSERVATIONS &&
        candidate->release_count <= MAX_OBSERVATIONS;
    if (valid) profile = *candidate;
    heap_caps_free(candidate);
    if (valid) {
      loaded = true;
      return true;
    }
  }

  LegacyProfileV2 *legacy = allocate_profile_buffer<LegacyProfileV2>();
  if (legacy == nullptr) {
    reset();
    return false;
  }
  blind_settings::FsFile file = blind_settings::fs_open(
      blind_settings::path_for(blind_settings::K_SOLAR_EXPOSURE_PROFILE), "r");
  if (file && file.size() == sizeof(*legacy) &&
      file.read(reinterpret_cast<uint8_t *>(legacy), sizeof(*legacy)) == sizeof(*legacy)) {
    file.close();
    if (legacy->magic == MAGIC && legacy->version == 2 &&
        legacy->start_count <= MAX_OBSERVATIONS && legacy->end_count <= MAX_OBSERVATIONS) {
      reset();
      profile.start_count = legacy->start_count;
      profile.release_count = legacy->end_count;
      std::copy(legacy->starts, legacy->starts + legacy->start_count, profile.starts);
      std::copy(legacy->ends, legacy->ends + legacy->end_count, profile.releases);
      heap_caps_free(legacy);
      return save();
    }
  } else if (file) {
    file.close();
  }

  heap_caps_free(legacy);
  reset();
  return false;
}

bool save() {
  return blind_settings::save(blind_settings::K_SOLAR_EXPOSURE_PROFILE, profile);
}

}  // namespace solar_exposure
