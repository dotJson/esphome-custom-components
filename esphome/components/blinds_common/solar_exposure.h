#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <type_traits>

namespace solar_exposure {

static constexpr uint32_t MAGIC = 0x534F4C52U;
static constexpr uint8_t VERSION = 3;
static constexpr uint8_t MAX_OBSERVATIONS = 48;
enum class Boundary : uint8_t { START = 0, LIMIT = 1, RELEASE = 2 };
enum class CapturePhase : uint8_t { IDLE = 0, BEGIN_CAPTURED = 1, LIMIT_CAPTURED = 2 };

struct Observation {
  uint16_t day_of_year{0};
  uint16_t local_minute{0};
  float azimuth{NAN};
  float elevation{NAN};
  uint32_t epoch{0};
};

struct Profile {
  uint32_t magic{MAGIC};
  uint8_t version{VERSION};
  uint8_t start_count{0};
  uint8_t limit_count{0};
  uint8_t release_count{0};
  Observation starts[MAX_OBSERVATIONS]{};
  Observation limits[MAX_OBSERVATIONS]{};
  Observation releases[MAX_OBSERVATIONS]{};
};

static_assert(std::is_trivially_copyable<Profile>::value, "Solar profile must remain POD");
inline Profile profile{};
inline bool loaded{false};
struct PendingCapture {
  CapturePhase phase{CapturePhase::IDLE};
  Observation begin{};
  Observation limit{};
};
inline PendingCapture pending_capture{};

inline void reset() { profile = Profile{}; loaded = true; }

bool load();
bool save();

inline uint8_t count(Boundary boundary) {
  return boundary == Boundary::START ? profile.start_count :
         boundary == Boundary::LIMIT ? profile.limit_count : profile.release_count;
}

inline const Observation *items(Boundary boundary) {
  return boundary == Boundary::START ? profile.starts :
         boundary == Boundary::LIMIT ? profile.limits : profile.releases;
}

inline uint32_t latest_capture_epoch(Boundary boundary) {
  const Observation *values = items(boundary);
  const uint8_t used = count(boundary);
  uint32_t latest = 0;
  for (uint8_t i = 0; i < used; i++) latest = std::max(latest, values[i].epoch);
  return latest;
}

// Report annual coverage as occupied half-month sectors. This rewards useful
// seasonal distribution instead of treating dense consecutive captures as a
// complete annual profile.
inline float coverage_percent(Boundary boundary) {
  bool occupied[24]{};
  const Observation *values = items(boundary);
  const uint8_t used = count(boundary);
  uint8_t occupied_count = 0;
  for (uint8_t i = 0; i < used; i++) {
    const uint8_t sector = std::min<uint8_t>(23, uint16_t(values[i].day_of_year - 1U) * 24U / 366U);
    if (!occupied[sector]) { occupied[sector] = true; occupied_count++; }
  }
  return 100.0f * float(occupied_count) / 24.0f;
}

// Recommend the centres of unrepresented half-month sectors, ordered from the
// next future opportunity. Callers can present these as reminder candidates.
inline uint8_t recommended_capture_days(Boundary boundary, uint16_t today,
                                        uint16_t *output, uint8_t capacity) {
  if (output == nullptr || capacity == 0) return 0;
  bool occupied[24]{};
  const Observation *values = items(boundary);
  for (uint8_t i = 0; i < count(boundary); i++) {
    const uint8_t sector = std::min<uint8_t>(23, uint16_t(values[i].day_of_year - 1U) * 24U / 366U);
    occupied[sector] = true;
  }
  uint8_t written = 0;
  const uint8_t current_sector = std::min<uint8_t>(23, uint16_t(std::max<uint16_t>(1, today) - 1U) * 24U / 366U);
  for (uint8_t offset = 1; offset <= 24 && written < capacity; offset++) {
    const uint8_t sector = (current_sector + offset) % 24;
    if (occupied[sector]) continue;
    const uint16_t begin = uint16_t((uint32_t(sector) * 366U) / 24U) + 1U;
    const uint16_t end = uint16_t((uint32_t(sector + 1U) * 366U) / 24U);
    output[written++] = uint16_t((uint32_t(begin) + end) / 2U);
  }
  return written;
}

inline int seasonal_distance(uint16_t a, uint16_t b) {
  const int direct = std::abs(int(a) - int(b));
  return std::min(direct, 366 - direct);
}

inline float angular_difference(float a, float b) {
  float delta = std::fabs(a - b);
  return delta > 180.0f ? 360.0f - delta : delta;
}

// Select the least valuable member of a full, sorted candidate set. A point is
// expendable when its two seasonal neighbours are close together and it lies
// near the curve they imply. Sharp bends receive a strong preservation bonus.
inline uint8_t least_informative(const Observation *values, uint8_t used) {
  uint8_t selected = 0;
  float lowest = INFINITY;
  for (uint8_t i = 0; i < used; i++) {
    const Observation &previous = values[(i + used - 1) % used];
    const Observation &current = values[i];
    const Observation &next = values[(i + 1) % used];
    const float left_gap = float(seasonal_distance(previous.day_of_year, current.day_of_year));
    const float right_gap = float(seasonal_distance(current.day_of_year, next.day_of_year));
    const float span = std::max(1.0f, left_gap + right_gap);
    const float fraction = left_gap / span;
    float az_delta = next.azimuth - previous.azimuth;
    if (az_delta > 180.0f) az_delta -= 360.0f;
    if (az_delta < -180.0f) az_delta += 360.0f;
    const float expected_azimuth = std::fmod(previous.azimuth + fraction * az_delta + 360.0f, 360.0f);
    const float expected_elevation = previous.elevation + fraction * (next.elevation - previous.elevation);
    const float curve_error = angular_difference(current.azimuth, expected_azimuth) +
                              1.5f * std::fabs(current.elevation - expected_elevation);
    // Small neighbour gaps make a point redundant; curve error protects useful
    // detail even when several samples happen to be close in date.
    const float value = std::min(left_gap, right_gap) + 4.0f * curve_error;
    if (value < lowest) { lowest = value; selected = i; }
  }
  return selected;
}

inline bool capture(Boundary boundary, uint16_t day, uint16_t minute,
                    float azimuth, float elevation, uint32_t epoch) {
  if (!loaded) load();
  if (day < 1 || day > 366 || minute > 1439 ||
      !std::isfinite(azimuth) || !std::isfinite(elevation)) return false;
  Observation *values = boundary == Boundary::START ? profile.starts :
                        boundary == Boundary::LIMIT ? profile.limits : profile.releases;
  uint8_t &used = boundary == Boundary::START ? profile.start_count :
                  boundary == Boundary::LIMIT ? profile.limit_count : profile.release_count;
  const Observation observation{day, minute, azimuth, elevation, epoch};
  for (uint8_t i = 0; i < used; i++) {
    if (values[i].day_of_year == day) { values[i] = observation; return save(); }
  }
  if (used < MAX_OBSERVATIONS) values[used++] = observation;
  else {
    Observation candidates[MAX_OBSERVATIONS + 1];
    std::copy(values, values + used, candidates);
    candidates[used] = observation;
    std::sort(candidates, candidates + used + 1, [](const Observation &a, const Observation &b) {
      return a.day_of_year < b.day_of_year;
    });
    const uint8_t discard = least_informative(candidates, used + 1);
    for (uint8_t source = 0, destination = 0; source < used + 1; source++) {
      if (source != discard) values[destination++] = candidates[source];
    }
  }
  std::sort(values, values + used, [](const Observation &a, const Observation &b) {
    return a.day_of_year < b.day_of_year;
  });
  return save();
}

inline void cancel_pending_capture() { pending_capture = PendingCapture{}; }

inline CapturePhase capture_phase(uint16_t today) {
  if (pending_capture.phase != CapturePhase::IDLE &&
      pending_capture.begin.day_of_year != today) cancel_pending_capture();
  return pending_capture.phase;
}

inline bool begin_capture(uint16_t day, uint16_t minute, float azimuth,
                          float elevation, uint32_t epoch) {
  if (day < 1 || day > 366 || minute > 1439 ||
      !std::isfinite(azimuth) || !std::isfinite(elevation)) return false;
  pending_capture = PendingCapture{};
  pending_capture.begin = Observation{day, minute, azimuth, elevation, epoch};
  pending_capture.phase = CapturePhase::BEGIN_CAPTURED;
  return true;
}

inline bool capture_limit(uint16_t day, uint16_t minute, float azimuth,
                          float elevation, uint32_t epoch) {
  if (capture_phase(day) != CapturePhase::BEGIN_CAPTURED ||
      minute <= pending_capture.begin.local_minute || minute > 1439 ||
      !std::isfinite(azimuth) || !std::isfinite(elevation)) return false;
  pending_capture.limit = Observation{day, minute, azimuth, elevation, epoch};
  pending_capture.phase = CapturePhase::LIMIT_CAPTURED;
  return true;
}

inline bool finish_capture(uint16_t day, uint16_t minute, float azimuth,
                           float elevation, uint32_t epoch) {
  if (capture_phase(day) != CapturePhase::LIMIT_CAPTURED ||
      minute <= pending_capture.limit.local_minute || minute > 1439 ||
      !std::isfinite(azimuth) || !std::isfinite(elevation)) return false;
  const Observation release{day, minute, azimuth, elevation, epoch};
  const Observation begin = pending_capture.begin;
  const Observation limit = pending_capture.limit;

  // Stage all three boundaries in RAM and persist the completed set once.
  // capture() is deliberately not used here because it saves each boundary.
  auto insert = [](Boundary boundary, const Observation &observation) {
    Observation *values = boundary == Boundary::START ? profile.starts :
                          boundary == Boundary::LIMIT ? profile.limits : profile.releases;
    uint8_t &used = boundary == Boundary::START ? profile.start_count :
                    boundary == Boundary::LIMIT ? profile.limit_count : profile.release_count;
    for (uint8_t i = 0; i < used; i++) {
      if (values[i].day_of_year == observation.day_of_year) {
        values[i] = observation;
        return;
      }
    }
    if (used < MAX_OBSERVATIONS) values[used++] = observation;
    else {
      Observation candidates[MAX_OBSERVATIONS + 1];
      std::copy(values, values + used, candidates);
      candidates[used] = observation;
      std::sort(candidates, candidates + used + 1, [](const Observation &a, const Observation &b) {
        return a.day_of_year < b.day_of_year;
      });
      const uint8_t discard = least_informative(candidates, used + 1);
      for (uint8_t source = 0, destination = 0; source < used + 1; source++)
        if (source != discard) values[destination++] = candidates[source];
    }
    std::sort(values, values + used, [](const Observation &a, const Observation &b) {
      return a.day_of_year < b.day_of_year;
    });
  };
  insert(Boundary::START, begin);
  insert(Boundary::LIMIT, limit);
  insert(Boundary::RELEASE, release);
  if (!save()) return false;
  cancel_pending_capture();
  return true;
}

inline float smoothstep(float value) {
  value = std::clamp(value, 0.0f, 1.0f);
  return value * value * (3.0f - 2.0f * value);
}

inline float winter_factor(uint16_t day, double latitude) {
  // Northern-hemisphere seasonal anchors, expressed as approximate day of
  // year: vernal equinox, summer solstice, autumnal equinox, winter solstice.
  // Shift half a year for southern-hemisphere installations.
  int seasonal_day = std::clamp<int>(day, 1, 366);
  if (latitude < 0.0) {
    seasonal_day += 182;
    if (seasonal_day > 366) seasonal_day -= 366;
  }

  constexpr int VERNAL_EQUINOX = 79;
  constexpr int SUMMER_SOLSTICE = 172;
  constexpr int AUTUMNAL_EQUINOX = 266;
  constexpr int WINTER_SOLSTICE = 355;

  if (seasonal_day >= SUMMER_SOLSTICE && seasonal_day < AUTUMNAL_EQUINOX) {
    return 0.0f;
  }
  if (seasonal_day >= AUTUMNAL_EQUINOX && seasonal_day < WINTER_SOLSTICE) {
    return smoothstep(float(seasonal_day - AUTUMNAL_EQUINOX) /
                      float(WINTER_SOLSTICE - AUTUMNAL_EQUINOX));
  }
  if (seasonal_day >= WINTER_SOLSTICE || seasonal_day < VERNAL_EQUINOX) {
    return 1.0f;
  }
  return 1.0f - smoothstep(float(seasonal_day - VERNAL_EQUINOX) /
                           float(SUMMER_SOLSTICE - VERNAL_EQUINOX));
}

inline float seasonal_position(float solar_position, float open_position, float winter_offset,
                               uint16_t day, double latitude) {
  solar_position = std::clamp(solar_position, 0.0f, 100.0f);
  open_position = std::clamp(open_position, 0.0f, 100.0f);
  const float distance_to_open = std::fabs(open_position - solar_position);
  const float adjustment = std::min(
      distance_to_open,
      std::max(0.0f, winter_offset) * winter_factor(day, latitude));
  if (solar_position < open_position) return solar_position + adjustment;
  if (solar_position > open_position) return solar_position - adjustment;
  return solar_position;
}

inline bool interpolate(Boundary boundary, uint16_t day, Observation &result) {
  if (!loaded) load();
  const Observation *values = items(boundary);
  const uint8_t used = count(boundary);
  if (used == 0) return false;
  if (used == 1) { result = values[0]; result.day_of_year = day; return true; }
  uint8_t after = 0;
  while (after < used && values[after].day_of_year < day) after++;
  const Observation &b = values[after % used];
  const Observation &a = values[(after + used - 1) % used];
  int ad = a.day_of_year, bd = b.day_of_year, target = day;
  if (bd <= ad) bd += 366;
  if (target < ad) target += 366;
  const float fraction = bd == ad ? 0.0f : float(target - ad) / float(bd - ad);
  result = a;
  result.day_of_year = day;
  result.local_minute = uint16_t(std::clamp<int>(
      int(std::lround(float(a.local_minute) + fraction * float(int(b.local_minute) - int(a.local_minute)))), 0, 1439));
  float da = b.azimuth - a.azimuth;
  if (da > 180.0f) da -= 360.0f;
  if (da < -180.0f) da += 360.0f;
  result.azimuth = std::fmod(a.azimuth + fraction * da + 360.0f, 360.0f);
  result.elevation = a.elevation + fraction * (b.elevation - a.elevation);
  return true;
}

inline bool predicted_minutes(Boundary boundary, uint16_t day, uint16_t &minute) {
  Observation estimate{};
  if (!interpolate(boundary, day, estimate)) return false;
  minute = estimate.local_minute;
  return true;
}

inline void coordinates(uint32_t epoch, double latitude, double longitude,
                        float &azimuth, float &elevation) {
  constexpr double DEG = 3.14159265358979323846 / 180.0;
  const double jd = double(epoch) / 86400.0 + 2440587.5;
  const double t = (jd - 2451545.0) / 36525.0;
  double l0 = std::fmod(280.46646 + t * (36000.76983 + 0.0003032 * t), 360.0);
  if (l0 < 0.0) l0 += 360.0;
  const double m = 357.52911 + t * (35999.05029 - 0.0001537 * t);
  const double e = 0.016708634 - t * (0.000042037 + 0.0000001267 * t);
  const double c = std::sin(m * DEG) * (1.914602 - t * (0.004817 + 0.000014 * t)) +
                   std::sin(2.0 * m * DEG) * (0.019993 - 0.000101 * t) +
                   std::sin(3.0 * m * DEG) * 0.000289;
  const double true_long = l0 + c;
  const double omega = 125.04 - 1934.136 * t;
  const double lambda = true_long - 0.00569 - 0.00478 * std::sin(omega * DEG);
  const double seconds = 21.448 - t * (46.815 + t * (0.00059 - t * 0.001813));
  const double obliq0 = 23.0 + (26.0 + seconds / 60.0) / 60.0;
  const double obliq = obliq0 + 0.00256 * std::cos(omega * DEG);
  const double decl = std::asin(std::sin(obliq * DEG) * std::sin(lambda * DEG));
  const double y = std::tan(obliq * DEG / 2.0) * std::tan(obliq * DEG / 2.0);
  const double eq_time = 4.0 / DEG * (y * std::sin(2.0 * l0 * DEG) -
      2.0 * e * std::sin(m * DEG) + 4.0 * e * y * std::sin(m * DEG) * std::cos(2.0 * l0 * DEG) -
      0.5 * y * y * std::sin(4.0 * l0 * DEG) - 1.25 * e * e * std::sin(2.0 * m * DEG));
  const uint32_t utc_day_seconds = epoch % 86400U;
  double solar_minutes = std::fmod(double(utc_day_seconds) / 60.0 + eq_time + 4.0 * longitude, 1440.0);
  if (solar_minutes < 0.0) solar_minutes += 1440.0;
  double hour_angle = solar_minutes / 4.0 - 180.0;
  const double lat = latitude * DEG;
  const double ha = hour_angle * DEG;
  const double cos_zenith = std::clamp(
      std::sin(lat) * std::sin(decl) + std::cos(lat) * std::cos(decl) * std::cos(ha), -1.0, 1.0);
  elevation = float(90.0 - std::acos(cos_zenith) / DEG);
  double az = std::atan2(std::sin(ha), std::cos(ha) * std::sin(lat) - std::tan(decl) * std::cos(lat)) / DEG + 180.0;
  azimuth = float(std::fmod(az + 360.0, 360.0));
}

inline bool predicted_minutes(Boundary boundary, uint16_t day, uint32_t local_midnight_epoch,
                              double latitude, double longitude, uint16_t &minute) {
  Observation target{};
  if (!interpolate(boundary, day, target)) return false;
  float best_score = INFINITY;
  uint16_t best_minute = target.local_minute;
  // The installation is west-facing; restricting the search prevents a
  // geometrically similar morning solution from being selected.
  for (uint16_t candidate = 10U * 60U; candidate < 24U * 60U; candidate++) {
    float az = NAN, el = NAN;
    coordinates(local_midnight_epoch + uint32_t(candidate) * 60U, latitude, longitude, az, el);
    float az_delta = std::fabs(az - target.azimuth);
    if (az_delta > 180.0f) az_delta = 360.0f - az_delta;
    const float el_delta = el - target.elevation;
    const float score = az_delta * az_delta + el_delta * el_delta * 2.0f;
    if (score < best_score) { best_score = score; best_minute = candidate; }
  }
  minute = best_minute;
  return std::isfinite(best_score);
}

inline bool window_for_day(uint16_t day, uint32_t local_midnight_epoch,
                           double latitude, double longitude,
                           uint16_t &start, uint16_t &limit, uint16_t &release) {
  if (!predicted_minutes(Boundary::START, day, local_midnight_epoch, latitude, longitude, start) ||
      !predicted_minutes(Boundary::RELEASE, day, local_midnight_epoch, latitude, longitude, release) ||
      start >= release) return false;
  if (!predicted_minutes(Boundary::LIMIT, day, local_midnight_epoch, latitude, longitude, limit)) {
    // Safe provisional behaviour for migrated profiles: reach the configured
    // limit one quarter of the way through the learned exposure window.
    limit = uint16_t(start + std::max<uint16_t>(1, uint16_t((release - start) / 4U)));
  }
  return start < limit && limit < release;
}

}  // namespace solar_exposure
