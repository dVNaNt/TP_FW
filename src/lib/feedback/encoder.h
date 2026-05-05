#pragma once  // Ensure this header is included only once during compilation

#include "../common.h"  // Include common migration definitions and types

/// Position integrator manages and calculates the absolute position from incremental encoder readings.
/// Uses full 32-bit left-aligned angle arithmetic to flawlessly track position without 64-bit integers.
struct Encoder {
private:
    // ---------------------------------------------------------------------------
    // Private data members
    // ---------------------------------------------------------------------------
    
    // 1) Угол энкодера: 32-бит без знака, выровнен влево (0x00000000 = 0°, 0xFFFFFFFF = ~360°)
    uint32_t angle_;       

    // 2) Счетчик полных оборотов: 32-бит со знаком
    int32_t rotations_;    

    friend struct Feedback;
    
public:
    // Constructor: Initialize position and rotations to zero
    Encoder() : angle_(0), rotations_(0) {}

    // Main processing function: Update position with new encoder reading
    // input_pos: Current encoder angle u32 (left aligned, full 32-bit resolution)
    inline void tick(uint32_t input_pos) {
        // Вычисляем разницу. Благодаря 32-битному переполнению, это всегда
        // правильный кратчайший путь, даже если мы перешли через 0.
        int32_t diff = (int32_t)(input_pos - angle_);
        
        // BRANCHLESS WRAP-AROUND DETECTION (Магия переполнения)
        // Нам нужно обновить счетчик оборотов. Условия:
        // (input_pos < angle_) даёт 1, если мы шагнули назад ИЛИ переполнились вперёд.
        // (diff < 0) даёт 1, если мы шагнули назад ИЛИ переполнились назад.
        // 
        // Вычитая одно булево значение из другого, мы идеально изолируем события перехода через ноль:
        // - Forward wrap:  input < angle (1) - diff < 0 (0) = +1 оборот
        // - Backward wrap: input < angle (0) - diff < 0 (1) = -1 оборот
        // - Normal fwd:    input < angle (0) - diff < 0 (0) =  0
        // - Normal bwd:    input < angle (1) - diff < 0 (1) =  0
        // 
        // Примечание: `(uint32_t)diff >> 31` - это ещё более быстрый способ получить бит знака (diff < 0).
        rotations_ += (input_pos < angle_) - ((uint32_t)diff >> 31);
        
        angle_ = input_pos;
    }

    // ---------------------------------------------------------------------------
    // Getters
    // ---------------------------------------------------------------------------

    // 1) Получить "чистый" угол (32 бита, левое выравнивание)
    inline uint32_t get_angle() const {
        return angle_;
    }

    // 2) Получить чистые полные обороты (32 бита, знаковые)
    inline int32_t get_rotations() const {
        return rotations_;
    }

    // 3) Combined position в формате Q16.16 (1 бит знак, 15 бит обороты, 16 бит угол).
    // Верхние 16 бит = обороты, нижние 16 бит = старшие 16 бит текущего угла.
    // Это полностью повторяет старую логику 32-битного position_, но с гораздо большей точностью внутри.
    inline int32_t get_position() const {
        return (int32_t)((rotations_ << 16) | (angle_ >> 16));
    }
};