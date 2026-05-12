#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

template <typename T>
class Protected {
  T _val;
  portMUX_TYPE _mux = portMUX_INITIALIZER_UNLOCKED;

public:
  void write(const T &v) {
    taskENTER_CRITICAL(&_mux);
    _val = v;
    taskEXIT_CRITICAL(&_mux);
  }
  T read() {
    taskENTER_CRITICAL(&_mux);
    T tmp = _val;
    taskEXIT_CRITICAL(&_mux);
    return tmp;
  }

  Protected() = default;
  Protected(const Protected &) = delete;
  Protected &operator=(const Protected &) = delete;
};
