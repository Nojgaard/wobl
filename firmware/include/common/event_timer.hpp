#pragma once

#include <Arduino.h>

class EventTimer {
public:
  struct Duration {
  public:
    static constexpr Duration fromUs(unsigned long us) { return Duration{us}; }

    static constexpr Duration fromMs(unsigned long ms) {
      return Duration{ms * 1000UL};
    }

    constexpr float seconds() const {
      return static_cast<float>(us) / 1'000'000.0f;
    }

    constexpr float hz() const { return us != 0UL ? 1.0f / seconds() : 0.0f; }

    constexpr bool blank() const { return us == 0UL; }

    unsigned long us;
  };

  void init() { _lastTickUs = micros(); }

  Duration poll(Duration period) {
    unsigned long nowUs = micros();
    unsigned long deltaUs = nowUs - _lastTickUs;

    if (deltaUs < period.us) {
      return Duration{0};
    }

    _lastTickUs = nowUs;
    return Duration{deltaUs};
  }

private:
  unsigned long _lastTickUs = 0;
};