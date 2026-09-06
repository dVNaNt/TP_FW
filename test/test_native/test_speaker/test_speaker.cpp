#include <gtest/gtest.h>
#include "lib/math_integer/helpers/speaker.h"

// ============================================================================
// MotorSpeaker tests
// ============================================================================

TEST(MotorSpeaker, InitialState) {
    MotorSpeaker speaker(20000);
    EXPECT_FALSE(speaker.is_playing());
    EXPECT_FALSE(speaker.is_finished());
    EXPECT_EQ(speaker.get_current_note(), 0);
}

TEST(MotorSpeaker, StartPlaying) {
    MotorSpeaker speaker(20000);
    speaker.start();
    EXPECT_TRUE(speaker.is_playing());
    EXPECT_FALSE(speaker.is_finished());
}

TEST(MotorSpeaker, StopPlaying) {
    MotorSpeaker speaker(20000);
    speaker.start();
    speaker.stop();
    EXPECT_FALSE(speaker.is_playing());
}

TEST(MotorSpeaker, TickReturnsZeroWhenNotPlaying) {
    MotorSpeaker speaker(20000);
    Vec2D result = speaker.tick();
    EXPECT_EQ(result.sin, 0);
    EXPECT_EQ(result.cos, 0);
}

TEST(MotorSpeaker, NoteProgression) {
    MotorSpeaker speaker(20000);
    speaker.start();
    uint8_t initial_note = speaker.get_current_note();

    // Tick through the first note's duration
    uint16_t first_note_duration = STARTUP_MELODY[0].duration;
    for (uint16_t i = 0; i < first_note_duration; i++) {
        speaker.tick();
    }

    // Should have advanced to the next note
    EXPECT_GT(speaker.get_current_note(), initial_note);
}

TEST(MotorSpeaker, MelodyCompletion) {
    MotorSpeaker speaker(20000);
    speaker.start();

    // Tick through entire melody
    uint32_t max_ticks = 0;
    for (int i = 0; i < MELODY_LENGTH; i++) {
        max_ticks += STARTUP_MELODY[i].duration;
    }

    for (uint32_t i = 0; i < max_ticks + 100; i++) {
        speaker.tick();
        if (speaker.is_finished()) break;
    }

    EXPECT_TRUE(speaker.is_finished());
    EXPECT_FALSE(speaker.is_playing());
}

TEST(MotorSpeaker, ProgressReporting) {
    MotorSpeaker speaker(20000);
    EXPECT_EQ(speaker.get_progress(), 0);

    speaker.start();
    // Progress should start at 0
    EXPECT_EQ(speaker.get_progress(), 0);
}

TEST(MotorSpeaker, SetAmplitude) {
    MotorSpeaker speaker(20000);
    speaker.set_amplitude(10000);
    speaker.start();
    // Should not crash, amplitude should be updated
}

TEST(MotorSpeaker, GetCurrentFrequency) {
    MotorSpeaker speaker(20000);
    speaker.start();
    // First note of STARTUP_MELODY is E5 (659 Hz)
    EXPECT_EQ(speaker.get_current_frequency(), STARTUP_MELODY[0].frequency);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
