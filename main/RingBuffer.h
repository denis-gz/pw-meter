#pragma once

#include <atomic>
#include <cstddef>

template <typename T, size_t Size>
class LockFreeRingBuffer
{
private:
    T buffer[Size];
    std::atomic<size_t> head {};
    std::atomic<size_t> tail {};

public:
    // Called strictly by the PRODUCER (Reader Task)
    // Returns true if an item was stored, false if buffer was full
    bool push(const T& item) {
        // We use relaxed (the "I don't care" flag) when a task is loading its own pointer
        // (the PRODUCER is the only task allowed to change the head pointer)
        size_t current_head = head.load(std::memory_order_relaxed);
        size_t next_head = (current_head + 1) % Size;

        // If the next head catches up to the tail, the buffer is full
        // We use acquire (the "Subscribe" flag) when a task checks the other task's pointer before doing work.
        if (next_head == tail.load(std::memory_order_acquire)) {
            return false; 
        }

        buffer[current_head] = item;
        // We use release (the "Publish" flag) when a task updates its pointer after doing work
        // (all data must be physically written to RAM before updating the pointer)
        head.store(next_head, std::memory_order_release);
        return true;
    }

    // Called strictly by the CONSUMER (Processing Task)
    // Returns true if an item was read, false if buffer was empty
    bool pop(T& item) {
        // We use relaxed ("I don't care" flag) when a task is loading its own pointer
        // (the CONSUMER is the only task allowed to change the tail pointer)
        size_t current_tail = tail.load(std::memory_order_relaxed);

        // If tail catches up to head, the buffer is empty
        // We use acquire (the "Subscribe" flag) when a task checks the other task's pointer before doing work.
        if (current_tail == head.load(std::memory_order_acquire)) {
            return false; 
        }

        item = buffer[current_tail];
        // We use release (the "Publish" flag) when a task updates its pointer after doing work
        // (all data must be physically written to RAM before updating the pointer)
        tail.store((current_tail + 1) % Size, std::memory_order_release);
        return true;
    }

    // Check how many items are available to read
    size_t available() const {
        size_t current_head = head.load(std::memory_order_acquire);
        size_t current_tail = tail.load(std::memory_order_acquire);
        
        if (current_head >= current_tail) {
            return current_head - current_tail;
        }
        return Size - current_tail + current_head;
    }
};
