#include "webhook_transport.h"

#include <new>
#include <esp_http_client.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

namespace async_webhook {

Transport transport;

bool Transport::busy() const {
  return busy_.load(std::memory_order_acquire);
}

bool Transport::submit(const std::string &url, const std::string &body, bool sample) {
  bool expected = false;
  if (!busy_.compare_exchange_strong(
        expected, true, std::memory_order_acq_rel, std::memory_order_acquire)) {
    return false;
  }

  result_ready_.store(false, std::memory_order_release);
  auto *job = new (std::nothrow) Job{this, url, body, sample};
  if (job == nullptr) {
    busy_.store(false, std::memory_order_release);
    return false;
  }

  TaskHandle_t handle = nullptr;
  const BaseType_t created = xTaskCreate(
    &Transport::task_entry_, "webhook_post", 8192, job, 1, &handle);

  if (created != pdPASS) {
    delete job;
    busy_.store(false, std::memory_order_release);
    return false;
  }
  return true;
}

bool Transport::consume_result(Result &out) {
  if (!result_ready_.exchange(false, std::memory_order_acq_rel)) return false;

  out.sample = result_sample_.load(std::memory_order_acquire);
  out.transport_ok = result_transport_ok_.load(std::memory_order_acquire);
  out.status_code = result_status_code_.load(std::memory_order_acquire);
  out.duration_ms = result_duration_ms_.load(std::memory_order_acquire);
  out.error = static_cast<esp_err_t>(result_error_.load(std::memory_order_acquire));
  busy_.store(false, std::memory_order_release);
  return true;
}

void Transport::task_entry_(void *arg) {
  auto *job = static_cast<Job *>(arg);
  Transport *self = job->owner;
  const int64_t started_us = esp_timer_get_time();

  esp_http_client_config_t config = {};
  config.url = job->url.c_str();
  config.timeout_ms = 2000;
  config.disable_auto_redirect = true;
  config.keep_alive_enable = false;

  esp_err_t err = ESP_FAIL;
  int status_code = -1;
  esp_http_client_handle_t client = esp_http_client_init(&config);

  if (client != nullptr) {
    esp_http_client_set_method(client, HTTP_METHOD_POST);
    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_post_field(
      client, job->body.c_str(), static_cast<int>(job->body.size()));
    err = esp_http_client_perform(client);
    if (err == ESP_OK) status_code = esp_http_client_get_status_code(client);
    esp_http_client_cleanup(client);
  }

  const uint32_t duration_ms = static_cast<uint32_t>(
    (esp_timer_get_time() - started_us) / 1000LL);

  self->result_sample_.store(job->sample, std::memory_order_release);
  self->result_transport_ok_.store(err == ESP_OK, std::memory_order_release);
  self->result_status_code_.store(status_code, std::memory_order_release);
  self->result_duration_ms_.store(duration_ms, std::memory_order_release);
  self->result_error_.store(static_cast<int>(err), std::memory_order_release);

  delete job;
  self->result_ready_.store(true, std::memory_order_release);
  vTaskDelete(nullptr);
}

}  // namespace async_webhook
