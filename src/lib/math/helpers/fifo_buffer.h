#pragma once // Ensures this header file is included only once during compilation

// Licensed under the Apache License, Version 2.0 // License declaration
// Copyright 2024 Anton Khrustalev, creapunk.com // Copyright notice

#include "../../common.h" // Includes common system-wide definitions and macros

#include <cstddef> // Includes size_t definition
#include <cstdint> // Includes fixed-width integer types
#include <utility> // Includes std::move for rvalue casting optimization

template <typename T, size_t N> // Template declaration: T is the data type, N is the total array size
class BufferFIFO { // Defines the lock-free ring buffer class
    // Compile-time check: A lock-free buffer without a count variable wastes one slot to distinguish full/empty states.
    // Therefore, the array size (N) must be at least 2 to hold at least 1 usable element.
    static_assert(N > 1, "Buffer size must be at least 2"); // Stops compilation if N is less than 2

private: // Private section for internal state variables
    T buffer[N]; // The underlying static array storing the actual elements
    
    // The 'volatile' keyword prevents the compiler from caching these variables in CPU registers,
    // ensuring that every read/write goes directly to RAM, which is critical for ISR communication.
    volatile size_t head = 0; // The index where the next new element will be written (modified ONLY by Producer)
    volatile size_t tail = 0; // The index from where the next element will be read (modified ONLY by Consumer)

public: // Public section for API methods
    [[nodiscard]] inline bool is_empty() const { // Checks if the buffer is empty; compiler warns if the result is ignored
        return head == tail; // The buffer is strictly empty when the head catches up to the tail
    } // Ends is_empty method
    
    [[nodiscard]] inline bool is_full() const { // Checks if the buffer is full; compiler warns if the result is ignored
        size_t next_head = head + 1; // Calculates the hypothetical next position for the head
        if (next_head >= N) next_head = 0; // Wraps the index back to 0 if it reaches the end of the array (ring logic)
        return next_head == tail; // The buffer is full if the next head position would hit the tail (leaving 1 slot empty)
    } // Ends is_full method

    [[nodiscard]] inline size_t get_count() const { // Calculates the current number of elements in the buffer
        size_t h = head; // Reads the volatile head into a local variable to prevent race conditions during calculation
        size_t t = tail; // Reads the volatile tail into a local variable to prevent race conditions during calculation
        if (h >= t) { // If the head is ahead of or equal to the tail (no wrap-around occurred)
            return h - t; // The count is simply the difference between head and tail
        } // Ends the if condition
        return N - t + h; // If wrapped around, the count is the remaining elements at the end plus elements at the start
    } // Ends get_count method

    inline void clear() { // Instantly resets the buffer to an empty state
        head = 0; // Resets the write pointer to the beginning
        tail = 0; // Resets the read pointer to the beginning, effectively dropping all logical data
    } // Ends clear method

    bool write(const T& item) { // Attempts to write a copy of an existing item (lvalue) into the buffer
        size_t current_head = head; // Caches the current head position locally
        size_t next_head = current_head + 1; // Calculates the next index
        if (next_head >= N) next_head = 0; // Wraps the index around if necessary

        if (next_head == tail) { // Checks if moving to the next head would overwrite unread data
            return false; // Returns false indicating the buffer is full and write failed
        } // Ends the full-check condition

        buffer[current_head] = item; // Copies the item into the array at the current head position
        
        // CRITICAL: The head is updated ONLY AFTER the data is fully written to memory.
        head = next_head; // Commits the new element, making it visible to the Consumer (ISR)
        return true; // Returns true indicating successful write
    } // Ends the lvalue write method

    bool write(T&& item) { // Attempts to move a temporary item (rvalue) into the buffer to save memory allocations
        size_t current_head = head; // Caches the current head position locally
        size_t next_head = current_head + 1; // Calculates the next index
        if (next_head >= N) next_head = 0; // Wraps the index around if necessary

        if (next_head == tail) { // Checks if the buffer is full
            return false; // Returns false indicating write failure
        } // Ends the full-check condition

        buffer[current_head] = std::move(item); // Moves the data into the array, transferring ownership efficiently
        
        head = next_head; // Updates the head, publishing the new data to the Consumer
        return true; // Returns true indicating successful write
    } // Ends the rvalue write method

    bool read(T& out_item) { // Attempts to extract an item from the buffer into the provided reference
        size_t current_tail = tail; // Caches the current tail position locally

        if (head == current_tail) { // Checks if the buffer is empty
            return false; // Returns false indicating there is no data to read
        } // Ends the empty-check condition

        out_item = std::move(buffer[current_tail]); // Moves the data out of the buffer into the receiver variable

        size_t next_tail = current_tail + 1; // Calculates the next tail index
        if (next_tail >= N) next_tail = 0; // Wraps the index around if necessary

        // CRITICAL: The tail is updated ONLY AFTER the data has been safely extracted.
        tail = next_tail; // Commits the read operation, freeing up the slot for the Producer
        return true; // Returns true indicating successful read
    } // Ends the read method

    inline bool peek(T& data) const { // Looks at the next available item without removing it from the buffer
        size_t current_tail = tail; // Caches the current tail position locally
        
        if (head == current_tail) { // Checks if the buffer is empty
            return false; // Returns false indicating no data is available
        } // Ends the empty-check condition
        
        data = buffer[current_tail]; // Copies the next item into the provided reference
        return true; // Returns true indicating successful peek
    } // Ends the peek method
}; // Ends the BufferFIFO class definition