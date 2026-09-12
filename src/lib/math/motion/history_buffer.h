/**
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */

// =============================================================================
// history_buffer.h (DYNAMIC HISTORY MODIFICATION)
// Ring buffer with history, reverse playback and clearing support.
// =============================================================================
#pragma once

#include <cstddef>
#include <cstdint>

template <typename T, size_t N>
class HistoryBuffer {
    static_assert(N >= 8 && (N & (N - 1)) == 0, "N must be power-of-two >= 8");
    static constexpr size_t MASK     = N - 1;
    static constexpr size_t MAX_HIST = N / 4; // Exactly 25% reserved for history under pressure

    T buf_[N];
    size_t head_   = 0;   
    size_t tail_   = 0;   
    size_t cursor_ = 0;   

    static inline size_t phys(size_t i) { return i & MASK; }

    void sanitize_state() {
        if (tail_ > head_)   tail_ = head_;
        if (cursor_ < tail_) cursor_ = tail_;
        if (cursor_ > head_) cursor_ = head_;
    }

    bool evict_for_write() {
        sanitize_state();
        
        // If history overgrew during idle (> 25%), force-shrink it.
        if (history_count() > MAX_HIST) {
            tail_ = cursor_ - 1 - MAX_HIST; 
        }

        // If space is available - write without eviction.
        if (head_ - tail_ < N) return true;

        // Evict history one element at a time to preserve the new trajectory.
        if (history_count() > 0) {
            ++tail_;
            return true;
        }

        // Buffer overflowed by future data. Write rejected.
        return false; 
    }

public:
    void clear() { head_ = tail_ = cursor_ = 0; }

    bool write(const T& item) {
        if (!evict_for_write()) {
            return false;
        }
        buf_[phys(head_++)] = item;
        return true;
    }

    inline bool has_future()  const { return cursor_ < head_; }
    inline bool has_active()  const { return cursor_ > tail_; }
    inline bool has_history() const { return cursor_ > tail_ + 1; }

    inline bool peek_future(T& out) const {
        if (cursor_ >= head_) return false;
        out = buf_[phys(cursor_)];
        return true;
    }

    inline bool peek_active(T& out) const {
        if (cursor_ <= tail_) return false;
        out = buf_[phys(cursor_ - 1)];
        return true;
    }

    inline bool peek_prev_history(T& out) const {
        if (cursor_ <= tail_ + 1) return false;
        out = buf_[phys(cursor_ - 2)];
        return true;
    }

    inline bool advance_cursor() {
        if (cursor_ >= head_) return false;
        ++cursor_;
        return true;
    }

    inline bool retreat_cursor(T& out_active) {
        if (cursor_ <= tail_ + 1) return false;
        
        --cursor_;
        out_active = buf_[phys(cursor_ - 1)];
        return true;
    }

    inline bool pop_active_to_future() {
        if (cursor_ <= tail_) return false;
        --cursor_;
        return true;
    }

    inline void trim_to_active()   { if (cursor_ > tail_) tail_ = cursor_ - 1; }
    inline void trim_all_history() { tail_ = cursor_; }

    /// @brief Free slots left for new segments (history is evicted before this hits zero).
    inline size_t free_count() const { return (head_ - tail_ >= N) ? 0 : (N - (head_ - tail_)); }

    inline size_t future_count() const { return head_ - cursor_; }
    inline size_t history_count() const { return (cursor_ > tail_) ? (cursor_ - tail_ - 1) : 0; }
};