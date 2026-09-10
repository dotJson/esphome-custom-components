#pragma once

// Asynchronous HTTP POST transport for webhook delivery.

#include <atomic>
#include <cstdint>
#include <string>
#include <esp_err.h>

namespace async_webhook {

struct Result {
  bool sample{false};
  bool transport_ok{false};
  int status_code{-1};
  uint32_t duration_ms{0};
  esp_err_t error{ESP_OK};
};

class Transport {
 public:
  bool busy() const;
  bool submit(const std::string &url, const std::string &body, bool sample);
  bool consume_result(Result &out);

 private:
  struct Job {
    Transport *owner;
    std::string url;
    std::string body;
    bool sample;
  };

  static void task_entry_(void *arg);

  std::atomic<bool> busy_{false};
  std::atomic<bool> result_ready_{false};
  std::atomic<bool> result_sample_{false};
  std::atomic<bool> result_transport_ok_{false};
  std::atomic<int> result_status_code_{-1};
  std::atomic<uint32_t> result_duration_ms_{0};
  std::atomic<int> result_error_{static_cast<int>(ESP_OK)};
};

extern Transport transport;

}  // namespace async_webhook
