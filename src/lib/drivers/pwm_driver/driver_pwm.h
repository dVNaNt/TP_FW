/**
 * @file pwm_driver.h
 * @brief Generic, high-performance PWM motor driver with integrated Current Sense,
 *        Voltage Sense, and unified fault management.
 *
 * @details
 * DriverPWM is the top-level driver structure that owns and coordinates:
 * - **CurrentSense** — 3-channel current sensing with overcurrent protection
 * - **SupplyVoltage** — filtered voltage sensing with OV/UV protection
 * - **Power** — power-stage power measurement (raw + EMA-filtered) with
 *   overpower (OP) protection from dual symmetric power/current limits
 * - **Temperature** — heavily-filtered temperature sensing with a single
 *   overtemperature (OT) threshold released by hysteresis
 * - **PwmControl** — hardware PWM output with chainable setup and SeqLock-protected duty cycle
 * - **OutputWrapper** — gate driver enable/disable
 * - **InputWrapper** — external hardware fault pin
 *
 * ## Fault Management Architecture
 *
 * The driver maintains a unified fault bitfield (@ref fault_flags_) with seven fault sources:
 *
 * | Bit | Fault         | Safe State | Recovery Timer  |
 * |-----|---------------|------------|-----------------|
 * | 0   | Overvoltage   | LowAll     | VoltageProtectionProxy |
 * | 1   | Overcurrent   | HiZ        | OvercurrentProxy |
 * | 2   | Overheat      | HiZ        | None (manual) |
 * | 3   | External      | HiZ        | None (manual) |
 * | 4   | Undervoltage  | HiZ        | VoltageProtectionProxy |
 * | 5   | Overpower     | HiZ        | Power::OverpowerProxy |
 * | 6   | Phase error   | HiZ        | None (follows MotorControl error state) |
 *
 * **Priority**: EXT (7) > OV (6) > UV (5) > OC (4) > PHASE (3) > OP (2) > OT (1). The highest-priority
 * active fault determines which callback fires. When all faults clear, the restore
 * callback fires.
 *
 * ## Thread Safety
 *
 * - PWM duty cycle updates use a SeqLock pattern inside PwmControl (ISR-safe writer, lock-free reader)
 * - Fault flags are atomic bitwise operations
 * - Active priority tracking uses compare_exchange to prevent race conditions
 *   between tick() and ISR-triggered fault changes
 *
 * ## Fluent API Examples
 *
 * PWM setup:
 * @code
 *   driver.pwm.set_period(7200).set_callback()
 *       .phases(hal_set_compare)
 *       .hiZ(hal_hiz)
 *       .lowAll(hal_low_all);
 * @endcode
 *
 * Fault callback configuration:
 * @code
 *   driver.set_fault().set_callback()
 *       .oc_cb(overcurrent_handler)
 *       .ov_cb(overvoltage_handler)
 *       .rst_cb(restore_handler);
 * @endcode
 *
 * Runtime duty cycle:
 * @code
 *   driver.pwm.set_duty(duty_q15);
 * @endcode
 *
 * @warning When any fault is active, tick_icr() skips PWM application entirely
 *          (the outputs remain in the hardware's safe-state configuration).
 *
 * @copyright Copyright 2026, creapunk
 * @license Apache-2.0
 */

#pragma once

#include <limits.h>
#include <stdint.h>
#include <atomic>

#include "../generic/input.h"
#include "../generic/output.h"
#include "current_sense.h"
#include "motor_control.h"
#include "power.h"
#include "pwm_control.h"
#include "temperature.h"
#include "voltage.h"

struct FaultCallbackProxy;
struct FaultTriggerProxy;

/**
 * @struct DriverPWM
 * @brief Generic PWM driver with built-in protection logic and fluent fault API.
 *
 * This is the primary driver object users instantiate. It aggregates current
 * sensing, voltage sensing, PWM output, and fault management into a single
 * cohesive interface.
 */
struct DriverPWM {
    friend struct CurrentSense;
    friend struct SupplyVoltage;
    friend struct MotorControl;
    friend struct Temperature;

public:
    // --- Fault bitfield (public: external code decodes get_fault()); safe states & priorities: see table above ---
    enum Fault : uint8_t {
        FAULT_OVERVOLTAGE = 1 << 0,   ///< LowAll (brake)
        FAULT_OVERCURRENT = 1 << 1,   ///< HiZ
        FAULT_OVERHEAT = 1 << 2,      ///< HiZ
        FAULT_EXTERNAL = 1 << 3,      ///< HiZ, highest priority
        FAULT_UNDERVOLTAGE = 1 << 4,  ///< HiZ
        FAULT_OVERPOWER = 1 << 5,     ///< HiZ
        FAULT_PHASE = 1 << 6,         ///< HiZ (MotorControl voltage/current path)
    };

private:
    // Fault-management proxies are defined at namespace scope below this struct
    friend struct FaultCallbackProxy;
    friend struct FaultTriggerProxy;

    std::atomic<uint8_t> fault_flags_ = 0;  ///< Bitfield holding current active faults

    int16_t pwm_apply_[3] = {0, 0, 0};  ///< Local array for applying PWM to hardware

    void (*cb_oc_)() = nullptr;       ///< Overcurrent fault callback
    void (*cb_ov_)() = nullptr;       ///< Overvoltage fault callback
    void (*cb_uv_)() = nullptr;       ///< Undervoltage fault callback
    void (*cb_ot_)() = nullptr;       ///< Overheat fault callback
    void (*cb_ext_)() = nullptr;      ///< External fault callback
    void (*cb_op_)() = nullptr;       ///< Overpower fault callback
    void (*cb_phase_)() = nullptr;    ///< Phase-stage error callback
    void (*cb_restore_)() = nullptr;  ///< Restore callback (fires when all faults clear)

    std::atomic<uint8_t> active_priority_ = 0;  ///< Tracks the highest active priority for callback logic

    /**
     * @brief Get the priority of the highest active fault.
     *
     * Priority encoding (higher number = higher priority):
     * - 7: External hardware fault
     * - 6: Overvoltage
     * - 5: Undervoltage
     * - 4: Overcurrent
     * - 3: Phase-stage error (voltage/current path)
     * - 2: Overpower (power/input-current limit)
     * - 1: Overheat
     * - 0: No fault active
     *
     * @param flags Current fault flags bitfield
     * @return Priority level (0..7)
     */
    inline uint8_t get_top_priority(uint8_t flags) const {
        if (flags & FAULT_EXTERNAL)
            return 7;
        if (flags & FAULT_OVERVOLTAGE)
            return 6;
        if (flags & FAULT_UNDERVOLTAGE)
            return 5;
        if (flags & FAULT_OVERCURRENT)
            return 4;
        if (flags & FAULT_PHASE)
            return 3;
        if (flags & FAULT_OVERPOWER)
            return 2;
        if (flags & FAULT_OVERHEAT)
            return 1;
        return 0;
    }

    /**
     * @brief Call the appropriate fault callback for a given priority level.
     *
     * Each callback is checked for nullptr before invocation — callbacks are
     * optional and may be left unset.
     *
     * @param priority Priority level (0=restore, 1=OT, 2=OP, 3=PHASE, 4=OC, 5=UV, 6=OV, 7=EXT)
     */
    inline void trigger_cb(uint8_t priority) {
        switch (priority) {
            case 7:
                if (cb_ext_)
                    cb_ext_();
                break;
            case 6:
                if (cb_ov_)
                    cb_ov_();
                break;
            case 5:
                if (cb_uv_)
                    cb_uv_();
                break;
            case 4:
                if (cb_oc_)
                    cb_oc_();
                break;
            case 3:
                if (cb_phase_)
                    cb_phase_();
                break;
            case 2:
                if (cb_op_)
                    cb_op_();
                break;
            case 1:
                if (cb_ot_)
                    cb_ot_();
                break;
            case 0:
                if (cb_restore_)
                    cb_restore_();
                break;
            default:
                break;
        }
    }

    /**
     * @brief Evaluate and trigger callback if a new higher-priority fault appeared.
     *
     * Uses compare_exchange to atomically promote the active priority only when
     * the new priority is strictly higher. This prevents race conditions between
     * tick() and ISR-triggered fault updates.
     *
     * @param new_flags The combined fault flags after the new fault was set
     */
    inline void check_and_trigger_priority(uint8_t new_flags) {
        uint8_t new_top = get_top_priority(new_flags);
        uint8_t expected = active_priority_.load(std::memory_order_relaxed);
        while (new_top > expected) {
            if (active_priority_.compare_exchange_weak(expected, new_top, std::memory_order_relaxed)) {
                trigger_cb(new_top);
                break;
            }
        }
    }

    /**
     * @brief Atomically set or clear a single fault bit without touching recovery timers.
     *
     * Used internally by tick_icr() to sync fault_flags_ with sub-system recovery
     * timer states. Does NOT invoke callbacks — that is handled by the priority manager.
     *
     * @param mask Fault bit mask (e.g., FAULT_OVERCURRENT)
     * @param active true to set the bit, false to clear it
     */
    inline void update_fault_bit(uint8_t mask, bool active) {
        if (active) {
            fault_flags_.fetch_or(mask, std::memory_order_relaxed);
        } else {
            fault_flags_.fetch_and(~mask, std::memory_order_relaxed);
        }
    }

public:
    MotorControl motor_ctrl;
    PwmControl pwm;           ///< PWM output stage with chainable setup and SeqLock-protected duty cycle
    OutputWrapper enable_;    ///< Digital output to enable/disable the gate driver
    InputWrapper error_;      ///< Digital input reading the hardware fault pin
    CurrentSense current;     ///< Public instance for current ADC input and configuration
    SupplyVoltage supply;     ///< Public instance for voltage ADC input and configuration
    Temperature temperature;  ///< Public instance for temperature ADC input, WT/OT protection
    Power power;              ///< Public instance for power measurement and OP (overpower) protection

    /**
     * @brief Construct a new DriverPWM object.
     *
     * Initializes all sub-systems: current sensing, voltage sensing,
     * temperature sensing, PWM output, enable pin, and error pin.
     *
     * @param ch1_fs_ma Full-scale current for Phase A in mA (0 = sensor missing)
     * @param ch2_fs_ma Full-scale current for Phase B in mA (0 = sensor missing)
     * @param ch3_fs_ma Full-scale current for Phase C in mA (0 = sensor missing)
     * @param v_alpha EMA filter alpha coefficient for voltage sensing (0..255)
     * @param v_max_mv Maximum voltage in millivolts for Q15 scaling
     * @param t_max_c Temperature in degC corresponding to the temperature
     *                channel's ADC full scale (Q15 normalization reference)
     */
    DriverPWM(float ch1_fs = 0,
              float ch2_fs = 0,
              float ch3_fs = 0,
              uint8_t v_alpha = 64,
              float v_max = 24.0f,
              float t_max_c = 150.0f)
        : motor_ctrl(), current(ch1_fs, ch2_fs, ch3_fs, motor_ctrl), supply(v_alpha, v_max), temperature(t_max_c) {
        // Wire the phase stage to its output backend and power estimator
        motor_ctrl.attach(pwm, power, supply);
    }

    /**
     * @brief Get reference to the enable output.
     * @return Reference to the OutputWrapper controlling the gate driver enable pin
     */
    inline OutputWrapper& get_enable() { return enable_; }

    /**
     * @brief Get reference to the error input.
     * @return Reference to the InputWrapper reading the hardware fault pin
     */
    inline InputWrapper& get_error() { return error_; }

    /**
     * @brief Get a FaultTriggerProxy for fluent fault triggering, clearing,
     *        and callback setup.
     *
     * This is the main entry point for interacting with the fault management
     * system.
     *
     * @return FaultTriggerProxy bound to this driver
     *
     * @par Example
     * @code
     *   // Set callbacks
     *   driver.set_fault().set_callback()
     *       .oc_cb(my_oc_handler)
     *       .rst_cb(my_restore_handler);
     *
     *   // Trigger/clear faults
     *   driver.set_fault().overcurrent(true);
     *   driver.set_fault().overcurrent(false);
     * @endcode
     */
    FaultTriggerProxy set_fault();

    /**
     * @brief Read the current fault flags bitfield.
     * @return Bitfield of currently active faults
     * @see FAULT_OVERVOLTAGE, FAULT_OVERCURRENT, FAULT_OVERHEAT, FAULT_EXTERNAL, FAULT_UNDERVOLTAGE, FAULT_OVERPOWER
     */
    inline uint8_t get_fault() const { return fault_flags_.load(std::memory_order_relaxed); }

    /**
     * @brief Sensor processing tick — processes current and voltage ADC data.
     *
     * Called before tick_icr(). Advances the EMA filter for voltage, runs
     * current normalization and digital overcurrent checks. Does NOT apply
     * PWM outputs or evaluate fault states.
     *
     * @note Call this first in your periodic ISR/scheduler, then call tick_icr().
     */
    inline void tick() {
        current.tick();
        supply.tick();
        temperature.tick();
        // Power stage: measured AB currents x commanded AB voltages, filtered
        // bus voltage from SupplyVoltage drives the input-current derivation
        power.tick(motor_ctrl.get_current_ab_amps(), supply.get_voltage());
    }

    /**
     * @brief Main logic tick — evaluates faults, manages priority, and applies PWM.
     *
     * Must be called periodically (e.g., every PWM cycle). Processing order:
     * 1. Sync fault flags with sub-system recovery timers (OC, OV, UV)
     * 2. Read external hardware fault pin
     * 3. Evaluate priority — fire callback if priority changed
     * 5. If any fault is active, skip PWM application (hardware stays in safe state)
     * 6. Otherwise, read duty cycle via SeqLock (PwmControl::read_ticks) and apply to hardware
     *
     * @note Call tick() before tick_icr() in each cycle.
     * @warning PWM outputs are NOT updated when any fault bit is set.
     */
    inline void tick_icr() {
        // 1. Sync sub-system recovery timer states with fault flags
        update_fault_bit(FAULT_OVERCURRENT, current.overcurrent.tick_timer());
        update_fault_bit(FAULT_OVERVOLTAGE, supply.overvoltage.tick_timer());
        update_fault_bit(FAULT_UNDERVOLTAGE, supply.undervoltage.tick_timer());
        // Power has a recovery latch: the flag follows the overpower proxy,
        // which trips on either symmetric limit (power or derived input
        // current) and auto-clears when its timer expires within limits.
        update_fault_bit(FAULT_OVERPOWER, power.overpower.tick_timer());
        // Phase stage: follows the MotorControl voltage/current path error
        // state of the last conversion tick (no recovery timer latch)
        update_fault_bit(FAULT_PHASE, motor_ctrl.is_err());
        // Temperature has no recovery latch: the flag follows the (heavily
        // filtered) measurement, trips at the OT limit and is released by
        // hysteresis once it drops below limit - hysteresis.
        update_fault_bit(FAULT_OVERHEAT, temperature.overheat.tick_timer());

        // 2. External hardware fault pin
        int ext = error_.check();
        if (ext != -1) {
            update_fault_bit(FAULT_EXTERNAL, ext);
        }

        // 3. Priority manager evaluation for tick updates (clears or software triggers)
        uint8_t flags = get_fault();
        uint8_t current_top = get_top_priority(flags);
        uint8_t expected_top = active_priority_.load(std::memory_order_relaxed);

        if (current_top != expected_top) {
            // Use compare_exchange to ensure we only trigger if state hasn't been changed by an interrupt
            if (active_priority_.compare_exchange_strong(expected_top, current_top, std::memory_order_relaxed)) {
                if (current_top == 0) {
                    if (cb_restore_)
                        cb_restore_();
                } else {
                    trigger_cb(current_top);
                }
            }
        }

        // 4. Apply safe state if any fault is active — do NOT output PWM
        if (flags != 0)
            return;

        // 5. Read duty cycle via SeqLock (now delegated to PwmControl)
        if (pwm.read_ticks(pwm_apply_)) {
            pwm.set_phases(pwm_apply_);
        }
    }
};

// ============================================================================
// FaultCallbackProxy — fluent fault-callback configuration
// ============================================================================
/**
 * @class FaultCallbackProxy
 * @brief Zero-overhead proxy for setting fault callbacks via fluent syntax.
 *
 * Defined at namespace scope so that forward declarations in this header and
 * friend declarations in sub-module headers (power.h, voltage.h,
 * current_sense.h) resolve to the same type as DriverPWM::set_fault().
 *
 * Usage:
 * @code
 *   driver.set_fault().set_callback()
 *       .oc_cb(overcurrent_handler)
 *       .ov_cb(overvoltage_handler)
 *       .ext_cb(external_fault_handler)
 *       .rst_cb(normal_op_restored_handler);
 * @endcode
 *
 * @note All setter methods safely accept nullptr (equivalent to unsetting the callback).
 */
struct FaultCallbackProxy {
    DriverPWM& drv_;  ///< Reference to the owning driver instance

    /**
     * @brief Construct a FaultCallbackProxy bound to a driver instance.
     * @param drv Reference to the owning DriverPWM
     */
    explicit FaultCallbackProxy(DriverPWM& drv) : drv_(drv) {}

    /**
     * @brief Set overcurrent fault callback.
     * @param cb Function pointer (nullptr to unset)
     * @return Reference to this proxy for chaining
     */
    inline FaultCallbackProxy& oc_cb(void (*cb)()) {
        drv_.cb_oc_ = cb;
        return *this;
    }
    /**
     * @brief Set overvoltage fault callback.
     * @param cb Function pointer (nullptr to unset)
     * @return Reference to this proxy for chaining
     */
    inline FaultCallbackProxy& ov_cb(void (*cb)()) {
        drv_.cb_ov_ = cb;
        return *this;
    }
    /**
     * @brief Set undervoltage fault callback.
     * @param cb Function pointer (nullptr to unset)
     * @return Reference to this proxy for chaining
     */
    inline FaultCallbackProxy& uv_cb(void (*cb)()) {
        drv_.cb_uv_ = cb;
        return *this;
    }
    /**
     * @brief Set overheat fault callback.
     * @param cb Function pointer (nullptr to unset)
     * @return Reference to this proxy for chaining
     */
    inline FaultCallbackProxy& ot_cb(void (*cb)()) {
        drv_.cb_ot_ = cb;
        return *this;
    }
    /**
     * @brief Set external fault callback.
     * @param cb Function pointer (nullptr to unset)
     * @return Reference to this proxy for chaining
     */
    inline FaultCallbackProxy& ext_cb(void (*cb)()) {
        drv_.cb_ext_ = cb;
        return *this;
    }
    /**
     * @brief Set overpower fault callback.
     * @param cb Function pointer (nullptr to unset)
     * @return Reference to this proxy for chaining
     */
    inline FaultCallbackProxy& op_cb(void (*cb)()) {
        drv_.cb_op_ = cb;
        return *this;
    }
    /**
     * @brief Set phase-stage error callback (MotorControl voltage/current path).
     * @param cb Function pointer (nullptr to unset)
     * @return Reference to this proxy for chaining
     */
    inline FaultCallbackProxy& phase_cb(void (*cb)()) {
        drv_.cb_phase_ = cb;
        return *this;
    }
    /**
     * @brief Set restore callback (fired when all faults clear).
     * @param cb Function pointer (nullptr to unset)
     * @return Reference to this proxy for chaining
     */
    inline FaultCallbackProxy& rst_cb(void (*cb)()) {
        drv_.cb_restore_ = cb;
        return *this;
    }
};

// ============================================================================
// FaultTriggerProxy — fluent fault triggering/clearing
// ============================================================================

/**
 * @class FaultTriggerProxy
 * @brief Zero-overhead proxy enabling fluent fault-triggering syntax.
 *
 * Defined at namespace scope so that forward declarations in this header and
 * friend declarations in sub-module headers (power.h, voltage.h,
 * current_sense.h) resolve to the same type as DriverPWM::set_fault().
 *
 * Any trigger(true) restarts the respective sub-system recovery timer.
 * Usage:
 * @code
 *   driver.set_fault().overcurrent(true);   // trigger + restart timer
 *   driver.set_fault().overcurrent(false);  // clear fault
 * @endcode
 */
struct FaultTriggerProxy {
    DriverPWM& drv_;  ///< Reference to the owning driver instance

    /**
     * @brief Construct a FaultTriggerProxy bound to a driver instance.
     * @param drv Reference to the owning DriverPWM
     */
    explicit FaultTriggerProxy(DriverPWM& drv) : drv_(drv) {}

    /**
     * @brief Get a FaultCallbackProxy for configuring fault callbacks.
     * @return FaultCallbackProxy bound to this driver
     */
    inline FaultCallbackProxy set_callback() { return FaultCallbackProxy(drv_); }

    /**
     * @brief Trigger or clear overvoltage fault (forces LowAll / brake).
     *
     * When triggered, restarts the overvoltage recovery timer in SupplyVoltage.
     *
     * @param state true to trigger (restarts timer), false to clear
     */
    inline void overvoltage(bool state) {
        if (state) {
            uint8_t old = drv_.fault_flags_.fetch_or(DriverPWM::FAULT_OVERVOLTAGE, std::memory_order_relaxed);
            drv_.supply.overvoltage.trigger();
            drv_.check_and_trigger_priority(old | DriverPWM::FAULT_OVERVOLTAGE);
        } else {
            drv_.fault_flags_.fetch_and(~DriverPWM::FAULT_OVERVOLTAGE, std::memory_order_relaxed);
            drv_.supply.overvoltage.reset();
        }
    }

    /**
     * @brief Trigger or clear undervoltage fault (forces HiZ).
     *
     * When triggered, restarts the undervoltage recovery timer in SupplyVoltage.
     *
     * @param state true to trigger (restarts timer), false to clear
     */
    inline void undervoltage(bool state) {
        if (state) {
            uint8_t old = drv_.fault_flags_.fetch_or(DriverPWM::FAULT_UNDERVOLTAGE, std::memory_order_relaxed);
            drv_.supply.undervoltage.trigger();
            drv_.check_and_trigger_priority(old | DriverPWM::FAULT_UNDERVOLTAGE);
        } else {
            drv_.fault_flags_.fetch_and(~DriverPWM::FAULT_UNDERVOLTAGE, std::memory_order_relaxed);
            drv_.supply.undervoltage.reset();
        }
    }

    /**
     * @brief Trigger or clear overcurrent fault (forces HiZ).
     *
     * When triggered, restarts the overcurrent recovery timer in CurrentSense.
     *
     * @param state true to trigger (restarts timer), false to clear
     */
    inline void overcurrent(bool state) {
        if (state) {
            uint8_t old = drv_.fault_flags_.fetch_or(DriverPWM::FAULT_OVERCURRENT, std::memory_order_relaxed);
            drv_.current.overcurrent.trigger();
            drv_.check_and_trigger_priority(old | DriverPWM::FAULT_OVERCURRENT);
        } else {
            drv_.fault_flags_.fetch_and(~DriverPWM::FAULT_OVERCURRENT, std::memory_order_relaxed);
            drv_.current.overcurrent.reset();
        }
    }

    /**
     * @brief Trigger or clear overpower fault (forces HiZ).
     *
     * When triggered, restarts the overpower recovery timer in Power.
     *
     * @param state true to trigger (restarts timer), false to clear
     */
    inline void overpower(bool state) {
        if (state) {
            uint8_t old = drv_.fault_flags_.fetch_or(DriverPWM::FAULT_OVERPOWER, std::memory_order_relaxed);
            drv_.power.overpower.trigger();
            drv_.check_and_trigger_priority(old | DriverPWM::FAULT_OVERPOWER);
        } else {
            drv_.fault_flags_.fetch_and(~DriverPWM::FAULT_OVERPOWER, std::memory_order_relaxed);
            drv_.power.overpower.reset();
        }
    }

    /**
     * @brief Trigger or clear the phase-stage error fault (forces HiZ).
     *
     * Mirrors the MotorControl voltage/current path error state. Normally
     * driven automatically by tick_icr(); exposed for manual override.
     *
     * @param state true to trigger, false to clear
     */
    inline void phase(bool state) {
        if (state) {
            uint8_t old = drv_.fault_flags_.fetch_or(DriverPWM::FAULT_PHASE, std::memory_order_relaxed);
            drv_.check_and_trigger_priority(old | DriverPWM::FAULT_PHASE);
        } else {
            drv_.fault_flags_.fetch_and(~DriverPWM::FAULT_PHASE, std::memory_order_relaxed);
        }
    }

    /**
     * @brief Trigger or clear overheat fault (forces HiZ).
     *
     * Overheat faults have no automatic recovery timer — they must be cleared
     * explicitly by the application.
     *
     * @param state true to trigger, false to clear
     */
    inline void overheat(bool state) {
        if (state) {
            uint8_t old = drv_.fault_flags_.fetch_or(DriverPWM::FAULT_OVERHEAT, std::memory_order_relaxed);
            drv_.check_and_trigger_priority(old | DriverPWM::FAULT_OVERHEAT);
        } else {
            drv_.fault_flags_.fetch_and(~DriverPWM::FAULT_OVERHEAT, std::memory_order_relaxed);
        }
    }

    /**
     * @brief Trigger or clear external hardware fault (forces HiZ, highest priority).
     *
     * External faults have no automatic recovery timer — they are linked to the
     * hardware fault pin state.
     *
     * @param state true to trigger, false to clear
     */
    inline void external(bool state) {
        if (state) {
            uint8_t old = drv_.fault_flags_.fetch_or(DriverPWM::FAULT_EXTERNAL, std::memory_order_relaxed);
            drv_.check_and_trigger_priority(old | DriverPWM::FAULT_EXTERNAL);
        } else {
            drv_.fault_flags_.fetch_and(~DriverPWM::FAULT_EXTERNAL, std::memory_order_relaxed);
        }
    }
};

/**
 * @brief Get a FaultTriggerProxy bound to this driver instance.
 * @return FaultTriggerProxy for fluent fault triggering/clearing/callback setup
 */
inline FaultTriggerProxy DriverPWM::set_fault() {
    return FaultTriggerProxy(*this);
}
