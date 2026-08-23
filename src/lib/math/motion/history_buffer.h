// =============================================================================
// history_buffer.h (DYNAMIC HISTORY MODIFICATION)
// Кольцевой буфер с историей, поддержкой реверса и очистки.
// =============================================================================
#pragma once

#include <cstddef>
#include <cstdint>

template <typename T, size_t N>
class HistoryBuffer {
    static_assert(N >= 8 && (N & (N - 1)) == 0, "N must be power-of-two >= 8");
    static constexpr size_t MASK     = N - 1;
    static constexpr size_t MAX_HIST = N / 4; // Ровно 25% под историю при конкуренции

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
        
        // Если история разрослась во время простоя (> 25%), принудительно сжимаем ее.
        if (history_count() > MAX_HIST) {
            tail_ = cursor_ - 1 - MAX_HIST; 
        }

        // Если место есть — пишем без вытеснения.
        if (head_ - tail_ < N) return true;

        // Вытесняем историю по одному элементу ради сохранения новой траектории.
        if (history_count() > 0) {
            ++tail_;
            return true;
        }

        // Буфер переполнен будущим. Запрет записи.
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

    inline size_t future_count() const { return head_ - cursor_; }
    inline size_t history_count() const { return (cursor_ > tail_) ? (cursor_ - tail_ - 1) : 0; }
};