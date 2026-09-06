#include <gtest/gtest.h>
#include "lib/math_integer/fifo_buffer.h"

// Declare a FIFO buffer type for testing
BUFFER_FIFO_DECLARE(TestFifo, int32_t, 4);

// ============================================================================
// FIFO buffer tests
// ============================================================================

TEST(FifoBuffer, NewBufferIsZeroed) {
    TestFifo fifo = TestFifo_new();
    // Reading should return 0 (default initialized)
    EXPECT_EQ(TestFifo_read(&fifo), 0);
}

TEST(FifoBuffer, WriteAndRead) {
    TestFifo fifo = TestFifo_new();
    TestFifo_write(&fifo, 100);
    // After write, index advances, so read points to next slot (which is 0)
    // The written value is at index 0, read is now at index 1
    EXPECT_EQ(TestFifo_read(&fifo), 0);

    // Write more to fill the buffer
    TestFifo_write(&fifo, 200);
    TestFifo_write(&fifo, 300);
    TestFifo_write(&fifo, 400);
    // Now index wrapped back to 0, reading should give the first write (100)
    EXPECT_EQ(TestFifo_read(&fifo), 100);
}

TEST(FifoBuffer, CircularWraparound) {
    TestFifo fifo = TestFifo_new();
    // Write 4 elements (fills entire buffer)
    TestFifo_write(&fifo, 10);
    TestFifo_write(&fifo, 20);
    TestFifo_write(&fifo, 30);
    TestFifo_write(&fifo, 40);
    // Index should be back at 0
    EXPECT_EQ(TestFifo_read(&fifo), 10);

    // Write one more (overwrites first element)
    TestFifo_write(&fifo, 50);
    EXPECT_EQ(TestFifo_read(&fifo), 20); // Next oldest element
}

TEST(FifoBuffer, PopReturnsOldAndWritesNew) {
    TestFifo fifo = TestFifo_new();
    TestFifo_write(&fifo, 100);
    TestFifo_write(&fifo, 200);
    TestFifo_write(&fifo, 300);
    TestFifo_write(&fifo, 400);
    // Index is at 0, pop should return buffer[0]=100 and write new value
    int32_t old = TestFifo_pop(&fifo, 999);
    EXPECT_EQ(old, 100);
    // Now index is at 1, reading should give buffer[1]=200
    EXPECT_EQ(TestFifo_read(&fifo), 200);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
