
// =============================================================================
// history_buffer.h
// Кольцевой буфер с историей, поддержкой реверса и очистки
// =============================================================================
#pragma once
#include <cstddef>
#include <cstdint>

// =============================================================================
// HistoryBuffer
//
// Single-reader/single-writer reversible ring buffer.
// Logical zones:
//   [ tail_ ... cursor_-2 ]  => Reversible history (completed segments)
//   [ cursor_-1 ]            => Currently active executing command
//   [ cursor_ ... head_-1 ]  => Future commands
// =============================================================================
template <typename T, size_t N>
class HistoryBuffer {
    static_assert(N >= 8 && (N & (N - 1)) == 0, "N must be power-of-two >= 8");
    static constexpr size_t MASK     = N - 1;
    static constexpr size_t MAX_HIST = N / 4;

    T buf_[N];
    volatile size_t head_   = 0;   
    volatile size_t tail_   = 0;   
    volatile size_t cursor_ = 0;   

    static __attribute__((always_inline)) size_t phys(size_t i) { return i & MASK; }

    __attribute__((always_inline))
    void sanitize_state() {
        if (tail_ > head_)   tail_ = head_;
        if (cursor_ < tail_) cursor_ = tail_;
        if (cursor_ > head_) cursor_ = head_;
    }

    __attribute__((always_inline))
    void evict_for_write() {
        sanitize_state();
        if (head_ - tail_ < N) return;

        if (cursor_ > tail_ + MAX_HIST) {
            ++tail_;
            return;
        }

        ++tail_;
        if (cursor_ < tail_) cursor_ = tail_;
    }

public:
    void clear() { head_ = tail_ = cursor_ = 0; }

    // Writes a new command into the future buffer.
    bool write(const T& item) {
        evict_for_write();
        if (head_ - tail_ >= N) return false;
        buf_[phys(head_++)] = item;
        return true;
    }

    __attribute__((always_inline)) bool has_future()  const { return cursor_ < head_; }
    __attribute__((always_inline)) bool has_active()  const { return cursor_ > tail_; }
    
    // History requires at least one completed segment before the active one
    __attribute__((always_inline)) bool has_history() const { return cursor_ > tail_ + 1; }

    __attribute__((always_inline))
    bool peek_future(T& out) const {
        if (cursor_ >= head_) return false;
        out = buf_[phys(cursor_)];
        return true;
    }

    __attribute__((always_inline))
    bool peek_active(T& out) const {
        if (cursor_ <= tail_) return false;
        out = buf_[phys(cursor_ - 1)];
        return true;
    }

    __attribute__((always_inline))
    bool peek_prev_history(T& out) const {
        if (cursor_ <= tail_ + 1) return false;
        out = buf_[phys(cursor_ - 2)];
        return true;
    }

    __attribute__((always_inline))
    bool advance_cursor() {
        if (cursor_ >= head_) return false;
        ++cursor_;
        if (head_ - tail_ >= N && cursor_ - tail_ > MAX_HIST) ++tail_;
        return true;
    }

    // Retreats the cursor backward in time.
    // Blocks and returns false if it hits a FREERUN barrier (INT32_MIN displacement).
    __attribute__((always_inline))
    bool retreat_cursor(T& out_active) {
        if (cursor_ <= tail_ + 1) return false;
        const size_t new_active = cursor_ - 2;
        
        // FREERUN barrier check: We cannot reverse into a relative movement without position.
        if (!buf_[phys(new_active)].has_displacement()) return false; 

        --cursor_;
        out_active = buf_[phys(cursor_ - 1)];
        return true;
    }

    // Pushes the currently active command back into the future zone.
    // Used when reverse playback hits the beginning of the active segment or a barrier.
    __attribute__((always_inline))
    bool pop_active_to_future() {
        if (cursor_ <= tail_) return false;
        --cursor_;
        return true;
    }

    __attribute__((always_inline)) void trim_to_active()   { if (cursor_ > tail_) tail_ = cursor_ - 1; }
    
    // Deletes all history before the cursor. Used when entering FREERUN.
    __attribute__((always_inline)) void trim_all_history() { tail_ = cursor_; }

    size_t future_count() const { return head_ - cursor_; }
    size_t history_count() const { return (cursor_ > tail_) ? (cursor_ - tail_ - 1) : 0; }
};
