#include <gtest/gtest.h>
#include "lib/feedback/error.h"

// ============================================================================
// ErrorManager tests
// ============================================================================

TEST(ErrorManager, InitialStateNoErrors) {
    ErrorManager em;
    EXPECT_FALSE(em.has_fatal());
    EXPECT_FALSE(em.has_alert());
    EXPECT_FALSE(em.has_warning());
    EXPECT_FALSE(em.has_any_error());
    EXPECT_EQ(em.get_error_level(), ErrorLevel_None);
    EXPECT_EQ(em.get_fatal_flags(), 0u);
    EXPECT_EQ(em.get_alert_flags(), 0u);
}

TEST(ErrorManager, SetFatalFlag) {
    ErrorManager em;
    mock_millis_set(100);
    em.set_fatal_dc_ovlo();
    EXPECT_TRUE(em.has_fatal());
    EXPECT_TRUE(em.get_fatal_dc_ovlo());
    EXPECT_FALSE(em.get_fatal_dc_uvlo()); // Other flags remain clear
    EXPECT_EQ(em.get_error_level(), ErrorLevel_Fatal);
}

TEST(ErrorManager, SetMultipleFatalFlags) {
    ErrorManager em;
    mock_millis_set(100);
    em.set_fatal_dc_ovlo();
    em.set_fatal_oc();
    EXPECT_TRUE(em.get_fatal_dc_ovlo());
    EXPECT_TRUE(em.get_fatal_oc());
    EXPECT_EQ(em.get_fatal_flags(), (uint32_t)(Fatal_DC_OVLO | Fatal_OC));
}

TEST(ErrorManager, SetAlertFlag) {
    ErrorManager em;
    mock_millis_set(200);
    em.set_alert_stall();
    EXPECT_TRUE(em.has_alert());
    EXPECT_TRUE(em.get_alert_stall());
    EXPECT_EQ(em.get_error_level(), ErrorLevel_Alert);
}

TEST(ErrorManager, ClearFatal) {
    ErrorManager em;
    mock_millis_set(100);
    em.set_fatal_dc_ovlo();
    em.clear_fatal();
    EXPECT_FALSE(em.has_fatal());
    EXPECT_FALSE(em.get_fatal_dc_ovlo());
    EXPECT_EQ(em.get_fatal_flags(), 0u);
}

TEST(ErrorManager, ClearAlert) {
    ErrorManager em;
    mock_millis_set(100);
    em.set_alert_dc_ovlo_soft();
    em.clear_alert();
    EXPECT_FALSE(em.has_alert());
    EXPECT_FALSE(em.get_alert_dc_ovlo_soft());
}

TEST(ErrorManager, ClearAll) {
    ErrorManager em;
    mock_millis_set(100);
    em.set_fatal_dc_ovlo();
    em.set_alert_stall();
    em.clear_all();
    EXPECT_FALSE(em.has_any_error());
}

TEST(ErrorManager, ErrorLevelHierarchy) {
    ErrorManager em;
    mock_millis_set(100);
    // No errors -> None
    EXPECT_EQ(em.get_error_level(), ErrorLevel_None);

    // Alert only -> Alert
    em.set_alert_stall();
    EXPECT_EQ(em.get_error_level(), ErrorLevel_Alert);

    // Fatal overrides alert
    em.set_fatal_overheat();
    EXPECT_EQ(em.get_error_level(), ErrorLevel_Fatal);
}

TEST(ErrorManager, AutoClearFatalAfterTimeout) {
    ErrorManager em;
    mock_millis_set(0);
    em.set_fatal_dc_ovlo();
    EXPECT_TRUE(em.has_fatal());

    // Before timeout (< 1000ms)
    mock_millis_set(999);
    em.tick();
    EXPECT_TRUE(em.has_fatal());

    // At timeout (>= 1000ms)
    mock_millis_set(1000);
    em.tick();
    EXPECT_FALSE(em.has_fatal());
}

TEST(ErrorManager, AutoClearAlertAfterTimeout) {
    ErrorManager em;
    mock_millis_set(0);
    em.set_alert_dc_ovlo_soft();
    EXPECT_TRUE(em.has_alert());

    mock_millis_set(1000);
    em.tick();
    EXPECT_FALSE(em.has_alert());
}

TEST(ErrorManager, AllFatalSetters) {
    ErrorManager em;
    mock_millis_set(100);

    em.set_fatal_dc_ovlo();
    EXPECT_TRUE(em.get_fatal_dc_ovlo());
    em.set_fatal_dc_uvlo();
    EXPECT_TRUE(em.get_fatal_dc_uvlo());
    em.set_fatal_phase1_oc();
    EXPECT_TRUE(em.get_fatal_phase1_oc());
    em.set_fatal_phase2_oc();
    EXPECT_TRUE(em.get_fatal_phase2_oc());
    em.set_fatal_phase3_oc();
    EXPECT_TRUE(em.get_fatal_phase3_oc());
    em.set_fatal_oc();
    EXPECT_TRUE(em.get_fatal_oc());
    em.set_fatal_current_leakage();
    EXPECT_TRUE(em.get_fatal_current_leakage());
    em.set_fatal_overheat();
    EXPECT_TRUE(em.get_fatal_overheat());
    em.set_fatal_no_load();
    EXPECT_TRUE(em.get_fatal_no_load());
    em.set_fatal_no_current_feedback();
    EXPECT_TRUE(em.get_fatal_no_current_feedback());
    em.set_fatal_emergency_stop();
    EXPECT_TRUE(em.get_fatal_emergency_stop());
    em.set_fatal_motor_error();
    EXPECT_TRUE(em.get_fatal_motor_error());
}

TEST(ErrorManager, AllAlertSetters) {
    ErrorManager em;
    mock_millis_set(100);

    em.set_alert_dc_ovlo_soft();
    EXPECT_TRUE(em.get_alert_dc_ovlo_soft());
    em.set_alert_dc_uvlo_soft();
    EXPECT_TRUE(em.get_alert_dc_uvlo_soft());
    em.set_alert_oc_soft();
    EXPECT_TRUE(em.get_alert_oc_soft());
    em.set_alert_overheat_soft();
    EXPECT_TRUE(em.get_alert_overheat_soft());
    em.set_alert_stall();
    EXPECT_TRUE(em.get_alert_stall());
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
