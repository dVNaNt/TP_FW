#pragma once

// Licensed under the Apache License, Version 2.0
// Copyright 2024 Anton Khrustalev, creapunk.com

#include "../common.h"

// A lightweight macro to declare a typed FIFO buffer equivalent to BufferFIFO<T, N>.
// Usage:
//   BUFFER_FIFO_DECLARE(MyFifo, int32_t, 16);
//   BUFFER_FIFO_INIT(MyFifo, fifo);
//   buffer_fifo_write(&fifo, value);
//   int32_t v = buffer_fifo_read(&fifo);
//
// The macro keeps the original semantics: a circular buffer with `write`, `read`, and `pop`.
#define BUFFER_FIFO_DECLARE(name, type, N)            \
    typedef struct {                                  \
        type buffer[N];                               \
        size_t idx;                                   \
    } name;                                           \
    static inline name name##_new(void) {             \
        name f;                                       \
        for (size_t i = 0; i < (N); ++i) {            \
            f.buffer[i] = (type){0};                  \
        }                                             \
        f.idx = 0;                                    \
        return f;                                     \
    }                                                 \
    static inline void name##_write(name* self, type value) { \
        self->buffer[self->idx] = value;              \
        self->idx = (self->idx + 1U) % (N);           \
    }                                                 \
    static inline type name##_read(const name* self) {\
        return self->buffer[self->idx];               \
    }                                                 \
    static inline type name##_pop(name* self, type value) { \
        type temp = self->buffer[self->idx];          \
        self->buffer[self->idx] = value;              \
        self->idx = (self->idx + 1U) % (N);           \
        return temp;                                  \
    }


