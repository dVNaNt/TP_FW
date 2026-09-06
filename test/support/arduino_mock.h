#pragma once
#ifndef ARDUINO
#include <stdint.h>
static uint32_t _mock_millis_value = 0;
static inline uint32_t millis() { return _mock_millis_value; }
static inline void mock_millis_set(uint32_t value) { _mock_millis_value = value; }
static inline void mock_millis_advance(uint32_t delta) { _mock_millis_value += delta; }
#endif
