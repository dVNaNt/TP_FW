#pragma once

// Licensed under the Apache License, Version 2.0
// Copyright 2024 Anton Khrustalev, creapunk.com

#include "../../common.h"



#include <cstdint>
#include <cstddef>
#include <utility> // Для std::move

template <typename T, size_t N>
class BufferFIFO {
private:
    T buffer[N];
    size_t head = 0;
    size_t tail = 0;
    size_t count = 0;

public:
    [[nodiscard]] bool is_empty() const { return count == 0; }
    [[nodiscard]] bool is_full() const { return count == N; }

    // Запись lvalue (копирование)
    bool write(const T& item) {
        if (is_full()) {
            return false; 
        }
        buffer[head] = item;
        
        head++;
        if (head >= N) head = 0; // Быстрая замена modulo (%) для произвольных N
        
        count++;
        return true;
    }

    // Запись rvalue (перемещение) - оптимизация для сложных структур и классов
    bool write(T&& item) {
        if (is_full()) {
            return false; 
        }
        buffer[head] = std::move(item);
        
        head++;
        if (head >= N) head = 0; 
        
        count++;
        return true;
    }

    // Чтение с извлечением (перемещением) данных
    bool read(T& out_item) {
        if (is_empty()) {
            return false;
        }
        
        // Перемещаем объект вместо копирования, чтобы избежать лишних аллокаций
        // внутри самого объекта (актуально для std::string, std::vector и тяжелых структур)
        out_item = std::move(buffer[tail]);
        
        tail++;
        if (tail >= N) tail = 0; 
        
        count--;
        return true;
    }
};
