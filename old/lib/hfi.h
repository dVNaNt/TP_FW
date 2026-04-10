                    // // INDUCTANCE MEASUREMENT SUB-STATE (HFI-based inductance detection) ////
                    // case MotorStatus::InductanceMeasure: {
                    //     // // Log start of inductance measurement on first cycle
                    //     // if (timeout.get_timeout() == INDUCTANCE_MEASURE_CYCLES) {  // First cycle
                    //     //     TP_LOG("[CALIB] Starting inductance measurement (1800 samples @ 1kHz HFI)");
                    //     //     TP_LOG_VAL("  HFI voltage amplitude: ", motor_calib.get_voltage());
                    //     //     TP_LOG_VAL("  Angle (D-axis): ", angle_);
                    //     // }

                    //     // // Generate 1 kHz HFI signal for D-axis injection
                    //     // Vec2D hfi_signal = hfi_generator_.tick();  // Get sine and cosine from generator

                    //     // // Apply HFI voltage to D-axis (cosine component for voltage injection)
                    //     // int16_t hfi_voltage = (motor_calib.get_voltage() * hfi_signal.cos * 25) >> 15;  // Scale by cosine
                    //     // (Q15) voltage_dq = Vec2D(hfi_voltage + motor_calib.get_voltage(), 0);  // Inject HFI voltage into
                    //     // D-axis only

                    //     // // Get current measurements from motor selector
                    //     // const int32_t curnt_a = motor_selector.get_currents().alpha;  // Phase A current
                    //     // const int32_t curnt_b = motor_selector.get_currents().beta;   // Phase B current

                    //     // // Demodulate HFI response using sine component (90° phase shift for inductive response)
                    //     // hfi_sense_a_.tick((int16_t)curnt_a, hfi_signal.cos);  // Demodulate phase A with sine reference
                    //     // hfi_sense_b_.tick((int16_t)curnt_b, hfi_signal.cos);  // Demodulate phase B with sine reference

                    //     // // Accumulate demodulated HFI responses for averaging
                    //     // int16_t hfi_response_a = hfi_sense_a_.get_output();                  // Get demodulated signal A
                    //     // int16_t hfi_response_b = hfi_sense_b_.get_output();                  // Get demodulated signal B
                    //     // // motor_calib.accumulate_inductance(hfi_response_a, hfi_response_b);  // Accumulate for averaging

                    //     // // Check if measurement complete (1800 cycles = 100 periods @ 1kHz)
                    //     // if (timeout.tick() == 0) {  // Counter reached 0 - measurement done
                    //     //     // Calculate inductance from averaged HFI response
                    //     //     // L = V / (2 * pi * f * I_hfi), where f = 1kHz
                    //     //     // Simplified: L_norm = HFI_response (proportional to inductance)
                    //     //     int32_t inductance_a = motor_calib.get_inductance_a();  // Get averaged inductance A
                    //     //     int32_t inductance_b = motor_calib.get_inductance_b();  // Get averaged inductance B

                    //     //     // Log inductance measurement results
                    //     //     TP_LOG("[CALIB] Inductance measurement complete:");
                    //     //     TP_LOG_VALS("  HFI response A/B (normalized): ", inductance_a, inductance_b);

                    //     //     // Store inductance values in motor structure (in microhenries, scaled from normalized)
                    //     //     // Conversion: L_uH = (V_norm * 1000) / (2 * pi * 1000 * I_norm) = V_norm / (6.28 * I_norm)
                    //     //     // Simplified proportional storage for now (actual calibration factor TBD)
                    //     //     motor.set_phase_inductance(0, inductance_a);  // Store phase A inductance
                    //     //     motor.set_phase_inductance(1, inductance_b);  // Store phase B inductance

                    //     //     TP_LOG_VALS("  Stored inductance A/B (uH): ", inductance_a, inductance_b);
                    //     //     TP_LOG("[CALIB] Motor calibration complete - transitioning to Ready");

                    //     //     // // Inductance measurement complete - transition to Ready state
                    //     //     // switch_status(DriverStatus_Ready);  // Calibration complete
                    //     //     // motor_calib.reset_voltage();       // Reset voltage to zero
                    //     //     // voltage_ab = Vec2D(0, 0);           // Zero output voltage
                    //     // }

    //                     // ========================================================================
    // // HFI Test Variables (Public for monitoring)
    // // ========================================================================
    // int16_t hfi_estimated_angle = 0;      // HFI estimated electrical angle (0-65535 = 0-360°)
    // int32_t hfi_estimated_velocity = 0;   // HFI estimated angular velocity (internal units)
    // int16_t hfi_injection_voltage = 0;    // Current HFI injection voltage (Q15)
    // int16_t hfi_measured_iq = 0;          // Measured Q-axis current for HFI (Q15)
    // Vec2D hfi_generator_signal = {0, 0};  // HFI generator sin/cos output (Q15)
    // bool hfi_active = false;              // Flag indicating HFI is running