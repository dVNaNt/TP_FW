#pragma once

/**
 * @file motor.h
 * @brief Motor description parameters, calibration lifecycle state and
 *        flash-backed parameter persistence.
 *
 * Motor holds the electrical/mechanical identity of the connected machine
 * (type, pole pairs, resistance/inductances, operating limits) plus the
 * MotorStatus state machine value describing where the automatic calibration
 * sequence currently is, and the encoder CalibrationTable used by the angle
 * calibrator (the whole Motor object is handed to the calibrator, which fills
 * the table in place).
 *
 * All characterizing parameters can be persisted into the USER-data flash
 * partition (FlashEEPROM instance injected via attach_flash(), wired to
 * TunePulseHeart::system_dump by the board layer):
 *
 * Stored-record word stream (records are appended slot by slot; the LAST
 * valid record with the requested Cal ID is the actual one):
 *   w0 : 0xCA1B | version<<16 | type<<26 | phase_conn<<28 | direction<<30
 *   w1 : cal_id | max_temp<<8 | pole_pairs<<16
 *   w2 : Resistance [15:0] | Inductance d [31:16]   (2 x bfloat16)
 *   w3 : Inductance q [15:0] | Max Torque [31:16]   (2 x bfloat16)
 *   w4 : Max Voltage [15:0] | Max Power [31:16]     (2 x bfloat16)
 *   w5 : Flux linkage [15:0] | Torque constant [31:16] (2 x bfloat16)
 *   w6 : Calibration offset (int32)
 *   w7 : Reserved (int32)
 *
 * The whole record fits EXACTLY one 8-word block (32 bytes), so records pack
 * back-to-back without any gaps. Both the load scan and the save write move
 * EXACTLY ONE 8-word block per service call, so a system tick never spends
 * more than one row-program/read inside flash (no blocking).
 *
 * SAVE GATE: the record embeds mechanical parameters (direction in w0,
 * cal_id/pole pairs in w1, angle-calibration offset in w6) that are only real
 * after the FULL calibration (electrical identification + angle/mech sweep)
 * has completed. begin_save() therefore refuses to arm until the angle
 * calibrator publishes set_mech_calibrated(true) - the identified electrical
 * parameters survive in RAM and are persisted by the calibrator's commit path
 * together with the fresh table reference.
 *
 * Copyright 2026, creapunk
 * Licensed under the Apache License, Version 2.0
 */

#include "../common.h"
#include "../drivers/flash_eeprom.h"
#include "types.h"
#include "calibration/calibration_table.h"

/** Command marker stored in the low half of the record header word. */
static constexpr uint16_t MOTOR_RECORD_CMD = 0xCA1B;

/** Serialized motor-type codes used inside the packed header word. */
enum : uint8_t {
    MOT_CODE_NONE = 0,  ///< Not defined.
    MOT_CODE_STEP = 1,  ///< STEPPER.
    MOT_CODE_BLDC = 2,  ///< BLDC.
    MOT_CODE_ERR  = 3   ///< ERROR marker (never written).
};

/** Serialized phase-connection codes used inside the packed header word. */
enum : uint8_t {
    PHASE_CODE_NONE = 0,  ///< Not connected.
    PHASE_CODE_ABC  = 1,  ///< ABC.
    PHASE_CODE_CAB  = 2,  ///< CAB (firmware CBA pattern).
    PHASE_CODE_ACB  = 3   ///< ACB.
};

/** Wildcard Cal-ID for begin_load(): accept any stored record. */
static constexpr uint8_t MOTOR_ID_ANY = 0xFF;

/**
 * @brief Pack a float into bfloat16 (1 sign / 8 exponent / 7 mantissa bits).
 *
 * Truncating conversion: the lowest 16 bits of the IEEE-754 representation
 * are dropped. Good enough for parameter storage (relative error <= 0.4%).
 */
inline uint16_t float_to_bfloat16(float value) {
    union {
        float f;
        uint32_t u;
    } conv;
    conv.f = value;
    return (uint16_t)(conv.u >> 16);
}

/**
 * @brief Unpack a bfloat16 word back into a float.
 */
inline float bfloat16_to_float(uint16_t raw) {
    union {
        float f;
        uint32_t u;
    } conv;
    conv.u = ((uint32_t)raw) << 16;
    return conv.f;
}

/**
 * Motor calibration status enum - accessible as SetupStatus::MotorCalibrationStatus.
 *
 * The automatic calibration sequence walks through these states:
 * SoftWarmup -> Settle -> SlowRotation -> ResistanceMeasure ->
 * InductanceMeasure(Warmup/D/Q/D2/Q2) -> ElectricCalibrationDone -> Success.
 */
enum class MotorStatus : uint8_t {
    Success = 0,            ///< Calibration finished / not calibrating.
    SoftWarmup = 1,         ///< Checking motor warmup and connection.
    Settle = 2,             ///< Settling phase before measurement (~900 ticks).
    SlowRotation = 3,       ///< Slow rotation for motor type detection.
    ResistanceMeasure = 4,  ///< Measuring winding resistance.
    InductanceMeasure = 5,  ///< Measuring inductance using HFI (high-frequency injection).
    CalibError = 6,         ///< Calibration error (abort).
    ElectricCalibrationDone = 7,   ///< Electrical identification finished.
    InductanceMeasureWarmup = 8,   ///< HFI warm-up stage.
    InductanceMeasureD  = 9,       ///< HFI measurement on the D axis (first pass).
    InductanceMeasureQ = 10,       ///< HFI measurement on the Q axis (first pass).
    InductanceMeasureD2  = 11,     ///< HFI measurement on the D axis (second pass).
    InductanceMeasureQ2 = 12,      ///< HFI measurement on the Q axis (second pass).
    // MechanicalCalibration = 8,  ///< (historical, unused)
    NotCalibrated = 127     ///< Initial state: no calibration has been run.
};

/**
 * Motor and driver system parameter container.
 *
 * Holds ALL parameters characterizing the motor (see the record layout in the
 * file header): type/pattern identity, pole pairs, resistance, D/Q inductance
 * and operating limits, plus runtime-only state (calibration status, angles,
 * current limit) and the encoder CalibrationTable used by the angle
 * calibrator.
 */
struct Motor {
private:
    // Motor type and configuration
    MotorType motor_type_;        ///< Type of motor (STEP / BLDC / Invalid).
    PhasePattern phase_pattern_;  ///< Phase connection pattern (ABC, ACB, CBA).
    MotorStatus status_;          ///< Current calibration status.

    uint16_t version_;        ///< Firmware version of the producing image (10 bit).
    uint8_t cal_id_;          ///< Calibration ID: chunk address (offset / 32) of the
                              ///< FA3E table record in the calibration partition.
    uint8_t max_temp_deg_c_;  ///< Maximum allowed temperature [deg C].

    // Motor mechanical parameters
    int32_t pole_count_;  ///< Number of POLE PAIRS (default 7 for a CLN17-class gimbal motor).
    int16_t direction_;   ///< Motor rotation direction (+1 = CW / -1 = CCW).

    // Motor angles (runtime only, never persisted)
    int32_t electric_angle_;  ///< Electric angle in internal fixed-point units.
    int32_t mechanic_angle_;  ///< Mechanical angle in internal fixed-point units.

    // Electrical parameters (persisted as bfloat16)
    float total_resistance_mohm_;  ///< Total (line-to-line equivalent) motor resistance [Ohm].
    float inductance_d_h_;         ///< D-axis inductance [H].
    float inductance_q_h_;         ///< Q-axis inductance [H].

    /// Default inductance used when no measurement exists [H].
    float default_inductance_h_;

    // Operating limits (persisted as bfloat16)
    float max_torque_nm_;         ///< Maximum torque [N*m].
    float max_voltage_v_;         ///< Maximum phase voltage [V].
    float max_power_w_;           ///< Maximum power [W].
    float flux_linkage_wb_;       ///< PM flux linkage [Wb].
    float torque_constant_;       ///< Torque constant Kt [N*m/A].
    int32_t calibration_offset_;  ///< Angle calibration offset value.
    int32_t reserved_;            ///< Reserved for future use.

    // Operating limits (runtime only)
    float current_limit_ma_;  ///< Current limit for calibration in milliamperes.

    // ------------------------------------------------------------------------
    // Flash persistence engine (system_dump partition).
    // Every load_tick()/save_tick() call moves exactly ONE 32-byte block
    // (8 x uint32_t words) - never more per system tick.
    // ------------------------------------------------------------------------
    enum FlashMode : uint8_t {
        FMODE_IDLE = 0,  ///< No transfer armed.
        FMODE_LOAD,      ///< Scanning the region for stored records.
        FMODE_SPACE,     ///< Locating free space for an append.
        FMODE_WRITE      ///< Streaming the record out block by block.
    };

    static constexpr uint32_t BLOCK_WORDS    = 8;            ///< Words per programmed/read block.
    static constexpr uint32_t RECORD_WORDS   = 8;            ///< Words in one parameter record (one block!).
    static constexpr uint32_t RECORD_BLOCKS  = 1;            ///< The whole record fits a single block.
    static constexpr uint32_t RECORD_BYTES   = 32;           ///< Bytes occupied by one record.
    static constexpr uint32_t ERASED_WORD32  = 0xFFFFFFFFul;  ///< Flash erase state.

    FlashEEPROM* flash_;              ///< Injected USER-data driver (nullptr = disabled).
    FlashMode f_mode_;                ///< Current engine mode.
    uint32_t f_addr_;                 ///< Byte cursor of the running scan/write.
    uint32_t f_free_;                 ///< First free byte offset found by the space scan.
    uint32_t f_word_;                 ///< Write stream cursor (words already streamed).
    bool load_ok_;                    ///< Last completed scan restored a valid record.
    bool store_ok_;                   ///< Last completed save wrote the record successfully.
    uint8_t load_id_;                 ///< Cal-ID filter of the running scan.
    uint32_t cand_buf_[RECORD_WORDS]; ///< Newest valid candidate seen by the scan.
    bool cand_valid_;                 ///< cand_buf_ holds a usable record.
    bool mech_calibrated_;            ///< Full (electrical + mech/angle) calibration done.
    bool save_pending_;               ///< A begin_save() was gated: re-arm when the
                                      ///< calibration table commits (set_mech_calibrated(true)).

public:
    /// Encoder calibration table - owned by the motor, filled by the calibrator.
    CalibrationTable cal_table;

    /// Constructor with default values (7 pole pairs, CW direction, 600 mA
    /// calibration current limit, 24 V max voltage, 50 W max power,
    /// 3 mH default inductance fallback).
    Motor()
        : motor_type_(MotorType_Invalid),
          phase_pattern_(PhasePattern_ABC),
          status_(MotorStatus::NotCalibrated),
          version_(1),
          cal_id_(0),
          max_temp_deg_c_(80),
          pole_count_(7),
          direction_(0),
          electric_angle_(0),
          mechanic_angle_(0),
          total_resistance_mohm_(0),
          inductance_d_h_(0),
          inductance_q_h_(0),
          default_inductance_h_(0.003f),  // Default 3 mH inductance fallback
          max_torque_nm_(0),
          max_voltage_v_(24.0f),
          max_power_w_(50.0f),
          flux_linkage_wb_(0),
          torque_constant_(0),
          calibration_offset_(0),
          reserved_(0),
          current_limit_ma_(600),
          flash_(nullptr),
          f_mode_(FMODE_IDLE),
          f_addr_(0),
          f_free_(0),
          f_word_(0),
          load_ok_(false),
          store_ok_(false),
          load_id_(MOTOR_ID_ANY),
          cand_valid_(false),
          mech_calibrated_(false),
          save_pending_(false),
          cal_table(CalibrationTable::create()) {
    }

    // === Setters ===

    /// Set motor type (enum overload).
    inline void set_motor_type(MotorType type) { motor_type_ = type; }
    /// Set motor type (raw numeric overload, cast to MotorType).
    inline void set_motor_type(int8_t type) { motor_type_ = (MotorType)type; }

    /// Set phase connection pattern.
    inline void set_phase_pattern(PhasePattern pattern) { phase_pattern_ = pattern; }

    /// Set calibration status.
    inline void set_status(MotorStatus status) { status_ = status; }

    /// Set firmware version stamp (10 bit, persisted in the header word).
    inline void set_version(uint16_t ver) { version_ = (uint16_t)(ver & 0x03FF); }

    /// Set the calibration-table sample ID (persisted with the record).
    inline void set_cal_id(uint8_t id) { cal_id_ = id; }

    /// Publish the full-calibration state: true only when the COMPLETE
    /// calibration (electrical identification + angle/mech sweep) has finished,
    /// so every mech field of the record (direction, cal_id, offset) is real.
    /// When a save was previously deferred by the begin_save() gate, it is
    /// re-armed HERE - exactly at the moment the calibration table record has
    /// been written to flash AND read back verified, so the profile reference
    /// (cal_id) always points at a table that is already committed. This also
    /// covers the reboot path where the boot restore finishes AFTER the
    /// electrical identification deferred its save and the stored table only
    /// commits later (boot restore branch, verify_after_save == false).
    inline void set_mech_calibrated(bool done) {
        mech_calibrated_ = done;
        if (done && save_pending_) {  // Deferred request waiting for this commit
            save_pending_ = false;    // Consume it exactly once
            begin_save();             // Re-arm with the now-real mech fields
        }
    }

    /// True once the full calibration completed and the profile record
    /// contains real (non-dummy) mechanical parameters.
    inline bool is_mech_calibrated() const { return mech_calibrated_; }

    /// Set maximum allowed temperature [deg C].
    inline void set_max_temp(uint8_t temp_deg_c) { max_temp_deg_c_ = temp_deg_c; }

    /// Set number of pole pairs.
    inline void set_pole_count(int32_t poles) { pole_count_ = poles; }
    /// Alias matching the record-sheet naming: number of pole pairs.
    inline void set_pole_pairs(int32_t pairs) { pole_count_ = pairs; }

    /// Set rotation direction (+1/-1).
    inline void set_direction(int16_t dir) { direction_ = dir; }

    /// Set electric angle (internal units).
    inline void set_electric_angle(int32_t angle) { electric_angle_ = angle; }

    /// Set mechanical angle (internal units).
    inline void set_mechanic_angle(int32_t angle) { mechanic_angle_ = angle; }

    // Set current limit for calibration
    inline void set_current_limit(float current_ma) { current_limit_ma_ = current_ma; }

    /// Set D-axis inductance [H] (as measured by HFI identification).
    inline void set_inductance_d(float l_d_h) { inductance_d_h_ = l_d_h; }

    /// Set Q-axis inductance [H] (as measured by HFI identification).
    inline void set_inductance_q(float l_q_h) { inductance_q_h_ = l_q_h; }

    /// Set maximum torque [N*m].
    inline void set_max_torque(float torque_nm) { max_torque_nm_ = torque_nm; }

    /// Set maximum voltage [V].
    inline void set_max_voltage(float voltage_v) { max_voltage_v_ = voltage_v; }

    /// Set maximum power [W].
    inline void set_max_power(float power_w) { max_power_w_ = power_w; }

    /// Set PM flux linkage [Wb].
    inline void set_flux_linkage(float flux_wb) { flux_linkage_wb_ = flux_wb; }

    /// Set torque constant Kt [N*m/A].
    inline void set_torque_constant(float kt) { torque_constant_ = kt; }

    /// Set angle calibration offset value (filled by the calibrator).
    inline void set_calibration_offset(int32_t offst) { calibration_offset_ = offst; }

    /// Override the fallback inductance value [H].
    inline void set_default_inductance(float inductance_h) { default_inductance_h_ = inductance_h; }

    /// Alias for set_total_resistance(): winding resistance [Ohm].
    inline void set_resistance(float resistance_ohm) { total_resistance_mohm_ = resistance_ohm; }

    // === Getters ===

    // Get motor type
    inline MotorType get_motor_type() const { return motor_type_; }

    // Get phase pattern
    inline PhasePattern get_phase_pattern() const { return phase_pattern_; }

    // Get driver status
    inline MotorStatus get_status() const { return status_; }

    // Get firmware version stamp
    inline uint16_t get_version() const { return version_; }

    // Get calibration-table sample ID
    inline uint8_t get_cal_id() const { return cal_id_; }

    // Get maximum allowed temperature [deg C]
    inline uint8_t get_max_temp() const { return max_temp_deg_c_; }

    // Get pole count (number of POLE PAIRS)
    inline int32_t get_pole_count() const { return pole_count_; }
    /// Alias matching the record-sheet naming: number of pole pairs.
    inline int32_t get_pole_pairs() const { return pole_count_; }

    // Get direction
    inline int16_t get_direction() const { return direction_; }

    // Get electric angle
    inline int32_t get_electric_angle() const { return electric_angle_; }

    // Get mechanical angle
    inline int32_t get_mechanic_angle() const { return mechanic_angle_; }

    // Get current limit for calibration
    inline float get_current_limit() const { return current_limit_ma_; }

    // Get D-axis inductance [H]
    inline float get_inductance_d() const { return inductance_d_h_; }

    // Get Q-axis inductance [H]
    inline float get_inductance_q() const { return inductance_q_h_; }

    /**
     * Get effective inductance (measured average or default fallback).
     *
     * Returns the mean of the measured Ld/Lq pair when available (> 0),
     * otherwise falls back to default_inductance_h_.
     */
    inline float get_effective_inductance() const {
        float measured = 0.0f;
        if (inductance_d_h_ > 0.0f && inductance_q_h_ > 0.0f) {
            measured = 0.5f * (inductance_d_h_ + inductance_q_h_);
        } else if (inductance_d_h_ > 0.0f) {
            measured = inductance_d_h_;
        } else if (inductance_q_h_ > 0.0f) {
            measured = inductance_q_h_;
        }
        if (measured > 0.0f) {
            return measured;
        }
        return default_inductance_h_;
    }

    // Get maximum torque [N*m]
    inline float get_max_torque() const { return max_torque_nm_; }

    // Get maximum voltage [V]
    inline float get_max_voltage() const { return max_voltage_v_; }

    // Get maximum power [W]
    inline float get_max_power() const { return max_power_w_; }

    // Get PM flux linkage [Wb]
    inline float get_flux_linkage() const { return flux_linkage_wb_; }

    // Get torque constant Kt [N*m/A]
    inline float get_torque_constant() const { return torque_constant_; }

    // Get angle calibration offset value
    inline int32_t get_calibration_offset() const { return calibration_offset_; }

    // Get total motor resistance [Ohm] (measured during identification)
    inline float get_total_resistance() const { return total_resistance_mohm_; }

    /// Alias for get_total_resistance(): winding resistance [Ohm].
    inline float get_resistance() const { return total_resistance_mohm_; }

    inline void set_total_resistance(float res) {
        total_resistance_mohm_ = res;
    }

private:
    // ========================================================================
    // Record serialization helpers
    // ========================================================================

    /// MotorType -> 2-bit serialized code.
    uint8_t type_code() const {
        switch (motor_type_) {
            case MotorType_STEP: return MOT_CODE_STEP;
            case MotorType_BLDC: return MOT_CODE_BLDC;
            default:             return MOT_CODE_NONE;
        }
    }

    /// PhasePattern -> 2-bit serialized code.
    uint8_t phase_code() const {
        switch (phase_pattern_) {
            case PhasePattern_ABC: return PHASE_CODE_ABC;
            case PhasePattern_CBA: return PHASE_CODE_CAB;
            case PhasePattern_ACB: return PHASE_CODE_ACB;
            default:               return PHASE_CODE_NONE;
        }
    }

    /// Rotation direction -> 2-bit serialized code (0 = CW, 1 = CCW).
    uint8_t dir_code() const { return (direction_ < 0) ? 1u : 0u; }

    /**
     * @brief Serialize the @p i -th word of the parameter record.
     * @param i Word index [0 .. RECORD_WORDS-1].
     */
    uint32_t stream_word(uint32_t i) {
        switch (i) {
            case 0:
                return (uint32_t)MOTOR_RECORD_CMD
                       | (((uint32_t)(version_ & 0x03FF)) << 16)
                       | (((uint32_t)type_code()) << 26)
                       | (((uint32_t)phase_code()) << 28)
                       | (((uint32_t)dir_code()) << 30);
            case 1:
                return (uint32_t)cal_id_
                       | (((uint32_t)max_temp_deg_c_) << 8)
                       | (((uint32_t)(pole_count_ & 0xFFFF)) << 16);
            // Packed bfloat16 pairs: low half = first parameter, high half = second.
            case 2:
                return (uint32_t)float_to_bfloat16(total_resistance_mohm_)
                       | (((uint32_t)float_to_bfloat16(inductance_d_h_)) << 16);
            case 3:
                return (uint32_t)float_to_bfloat16(inductance_q_h_)
                       | (((uint32_t)float_to_bfloat16(max_torque_nm_)) << 16);
            case 4:
                return (uint32_t)float_to_bfloat16(max_voltage_v_)
                       | (((uint32_t)float_to_bfloat16(max_power_w_)) << 16);
            case 5:
                return (uint32_t)float_to_bfloat16(flux_linkage_wb_)
                       | (((uint32_t)float_to_bfloat16(torque_constant_)) << 16);
            case 6:  return (uint32_t)calibration_offset_;
            default: return (uint32_t)reserved_;
        }
    }

    /// Quick header sanity check on a staged candidate record.
    static bool header_valid(const uint32_t* r) {
        if ((r[0] & 0xFFFFu) != MOTOR_RECORD_CMD) {
            return false;  // Wrong command marker
        }
        uint8_t tcode = (uint8_t)((r[0] >> 26) & 0x3u);
        if (tcode == MOT_CODE_ERR || tcode == MOT_CODE_NONE) {
            return false;  // Type never identified
        }
        uint32_t poles = (r[1] >> 16) & 0xFFFFu;
        if (poles == 0 || poles > 4095u) {
            return false;  // Pole pairs out of any plausible range
        }
        return true;
    }

    /// Apply a validated record into the live parameter fields.
    void apply_record(const uint32_t* r) {
        version_ = (uint16_t)((r[0] >> 16) & 0x03FFu);

        switch ((r[0] >> 26) & 0x3u) {  // Serialized motor type -> enum
            case MOT_CODE_STEP: motor_type_ = MotorType_STEP; break;
            case MOT_CODE_BLDC: motor_type_ = MotorType_BLDC; break;
            default:            motor_type_ = MotorType_Invalid; break;
        }

        switch ((r[0] >> 28) & 0x3u) {  // Serialized phase connection -> pattern
            case PHASE_CODE_ABC: phase_pattern_ = PhasePattern_ABC; break;
            case PHASE_CODE_CAB: phase_pattern_ = PhasePattern_CBA; break;
            case PHASE_CODE_ACB: phase_pattern_ = PhasePattern_ACB; break;
            default:             break;  // Keep current pattern for "not connected"
        }

        direction_ = ((r[0] >> 30) & 0x1u) ? (int16_t)-1 : (int16_t)1;

        cal_id_         = (uint8_t)(r[1] & 0xFFu);
        max_temp_deg_c_ = (uint8_t)((r[1] >> 8) & 0xFFu);
        pole_count_     = (int32_t)((r[1] >> 16) & 0xFFFFu);

        total_resistance_mohm_ = bfloat16_to_float((uint16_t)(r[2] & 0xFFFFu));
        inductance_d_h_        = bfloat16_to_float((uint16_t)(r[2] >> 16));
        inductance_q_h_        = bfloat16_to_float((uint16_t)(r[3] & 0xFFFFu));
        max_torque_nm_         = bfloat16_to_float((uint16_t)(r[3] >> 16));
        max_voltage_v_         = bfloat16_to_float((uint16_t)(r[4] & 0xFFFFu));
        max_power_w_           = bfloat16_to_float((uint16_t)(r[4] >> 16));
        flux_linkage_wb_       = bfloat16_to_float((uint16_t)(r[5] & 0xFFFFu));
        torque_constant_       = bfloat16_to_float((uint16_t)(r[5] >> 16));

        calibration_offset_ = (int32_t)r[6];
        reserved_           = (int32_t)r[7];
    }

    /// Print the active parameter set to serial (engineering units so small
    /// values like inductances stay visible - Arduino prints only 2 digits).
    void report(bool loaded) {
        TP_LOG(loaded ? "MOTOR: parameters restored from system_dump:"
                      : "MOTOR: parameters saved to system_dump:");
        TP_LOG_VALS("  ver/type/pattern/dir", (unsigned)version_, (unsigned)type_code(),
                    (unsigned)phase_code(), (direction_ < 0) ? "CCW" : "CW");
        TP_LOG_VALS("  cal_id/chunk,max_temp,pole_pairs", (unsigned)cal_id_,
                    (unsigned)max_temp_deg_c_, (long)pole_count_);
        TP_LOG_VALS("  R[mOhm],Ld[uH],Lq[uH]",
                    total_resistance_mohm_ * 1000.0f,
                    inductance_d_h_ * 1e6f,
                    inductance_q_h_ * 1e6f);
        TP_LOG_VALS("  Tmax[mNm],V_max[V],P_max[W]",
                    max_torque_nm_ * 1000.0f, max_voltage_v_, max_power_w_);
        TP_LOG_VALS("  flux[uWb],Kt[mNm/A],cal_offset",
                    flux_linkage_wb_ * 1e6f,
                    torque_constant_ * 1000.0f,
                    (long)calibration_offset_);
    }

public:
    // ========================================================================
    // Flash persistence API (system_dump): the two transfer methods
    // ========================================================================

    /// Bind the USER-data flash driver used for parameter persistence.
    void attach_flash(FlashEEPROM* mem) { flash_ = mem; }

    /// True while the record engine is mid-transfer (load scan / space scan / write).
    /// Each write tick programs 32 bytes and stalls the single-bank flash for
    /// several PWM periods, so the control loop de-energizes while this is set.
    bool is_flash_busy() const { return f_mode_ != FMODE_IDLE; }

    /// True when a completed scan restored a valid record.
    bool is_loaded() const { return load_ok_; }

    /// True when the last save wrote the record successfully.
    bool is_saved() const { return store_ok_; }

    /**
     * @brief Arm the non-blocking LOAD scan (method #1: "read").
     *
     * Scans the whole partition one 8-word block per load_tick() call and
     * keeps the NEWEST valid record whose Cal-ID matches @p id. If something
     * was already written - that newest matching record wins.
     *
     * @param id Cal-ID to match, or MOTOR_ID_ANY to accept any record.
     */
    void begin_load(uint8_t id) {
        load_ok_    = false;
        cand_valid_ = false;
        load_id_    = id;
        f_addr_     = 0;
        f_mode_     = flash_ ? FMODE_LOAD : FMODE_IDLE;
    }

    /// begin_load() overload filtering on the currently stored cal_id.
    void begin_load() { begin_load(cal_id_); }

private:
    /// Commit the best candidate found by the scan and report over serial.
    void finish_scan(const char* fault_msg) {
        f_mode_ = FMODE_IDLE;
        if (cand_valid_) {
            apply_record(cand_buf_);
            load_ok_ = true;
            report(true);
        } else {
            TP_LOG(fault_msg);
        }
    }

public:
    /**
     * @brief Advance the LOAD scan by exactly one 8-word block.
     * @return true when the engine is finished (idle or done), false while
     *         more blocks remain to be scanned.
     */
    bool load_tick() {
        if (f_mode_ != FMODE_LOAD) {
            return true;
        }
        if (!flash_ || flash_->get_max_size() == 0) {
            return false;  // Driver not bound/configured yet: retry later
        }
        uint32_t region = flash_->get_max_size();

        if (f_addr_ + BLOCK_WORDS * 4 > region) {
            finish_scan("MOTOR: no stored parameters found in system_dump");
            return true;  // Region exhausted: newest candidate wins
        }

        uint32_t w[BLOCK_WORDS];
        if (!flash_->read_block(f_addr_, w)) {
            finish_scan("MOTOR: system_dump read fault during scan");
            return true;  // Hardware fault: keep whatever candidate exists
        }
        f_addr_ += BLOCK_WORDS * 4;

        // Every block holds one WHOLE record (RECORD_WORDS == BLOCK_WORDS):
        // validate it directly. Later valid records overwrite earlier ones
        // in the candidate slot => the newest valid record wins.
        if (w[0] != ERASED_WORD32 && header_valid(w)) {
            uint8_t rec_id = (uint8_t)(w[1] & 0xFFu);
            if (load_id_ == MOTOR_ID_ANY || rec_id == load_id_) {
                for (uint32_t i = 0; i < RECORD_WORDS; ++i) {
                    cand_buf_[i] = w[i];
                }
                cand_valid_ = true;
            }
        }
        return false;
    }

    /**
     * @brief Arm the non-blocking SAVE (method #2: "write all parameters").
     *
     * Tries to save the whole motor parameter set: locates a free 32-byte
     * slot (one block-read per tick), then appends the whole 8-word record
     * with ONE block write (one per tick); erases + rewrites from offset zero
     * when the partition is exhausted.
     */
    void begin_save() {
        store_ok_ = false;
        if (!mech_calibrated_) {
            // Record gate: w0 embeds the measured rotation direction, w1 the
            // cal_id/pole pairs and w6 the angle-calibration offset - all of
            // them are still dummy/default values until the FULL calibration
            // (including the angle/mech sweep) commits its table. Never
            // persist such a half-valid record: remember the request and
            // re-arm it exactly when set_mech_calibrated(true) reports that
            // the fresh table has been written to flash AND read back
            // verified, so the cal_id reference can never point at a
            // not-yet-written (or failed) table record.
            save_pending_ = true;
            TP_LOG("MOTOR: save deferred - full calibration not complete");
            return;
        }
        if (!flash_ || flash_->get_max_size() == 0) {
            return;  // Nothing bound yet: silently ignore
        }
        f_free_ = flash_->get_max_size();  // Default: no free space found
        f_addr_ = 0;
        f_word_ = 0;
        f_mode_ = FMODE_SPACE;
    }

    /**
     * @brief Advance the SAVE engine by exactly one 8-word read OR write.
     *
     * Phase SPACE walks the partition in 32-byte record SLOTS (one block-read
     * per tick) and stops at the first slot whose leading word is erased.
     * Phase WRITE streams the whole 8-word record with a single block write;
     * erases + rewrites from offset zero when no free slot remains.
     *
     * @return true when the engine is finished (idle or done).
     */
    bool save_tick() {
        if (f_mode_ == FMODE_SPACE) {
            uint32_t region = flash_->get_max_size();
            if (region == 0) {
                f_mode_ = FMODE_IDLE;
                return true;
            }

            uint32_t w[BLOCK_WORDS];
            if (f_addr_ + RECORD_BYTES > region) {
                f_mode_ = FMODE_IDLE;  // Region exhausted: no free slot found
            } else if (!flash_->read_block(f_addr_, w)) {
                // Read fault == never-programmed (erased) block: erased flash
                // raises a double-bit ECC NMI and read_section() reports false.
                // A free slot therefore starts RIGHT HERE - appending avoids
                // the needless erase-and-rewrite of the whole partition
                // (flash wear + a tens-of-ms __disable_irq() stall inside the
                // control ISR on every save).
                f_free_ = f_addr_;
                f_mode_ = FMODE_IDLE;
            } else {
                if (w[0] == ERASED_WORD32) {
                    f_free_ = f_addr_;  // Leading word erased: whole slot is free
                    f_mode_ = FMODE_IDLE;
                } else {
                    f_addr_ += RECORD_BYTES;  // Occupied slot: advance to the next one
                }
            }
            if (f_mode_ != FMODE_IDLE) {
                return false;  // Still scanning
            }

            if (f_free_ <= region - RECORD_BYTES) {
                f_word_ = 0;
                f_mode_ = FMODE_WRITE;  // Append into the free slot
            } else if (!tp_flash_programming_window) {
                // Partition full: the whole-region erase stalls the ISR with
                // IRQs disabled for tens of ms. Wait for the de-energized
                // window (zero-current vector latched) before wiping.
                f_mode_ = FMODE_SPACE;  // Re-evaluate the decision on the next tick
            } else if (flash_->erase(0xDEADC0DE)) {
                TP_LOG("MOTOR: system_dump full - erasing and rewriting");
                f_free_ = 0;
                f_word_ = 0;
                f_mode_ = FMODE_WRITE;
            } else {
                TP_LOG("MOTOR: system_dump save failed (no space / erase error)");
                return true;
            }
            return f_mode_ != FMODE_WRITE;
        }

        if (f_mode_ == FMODE_WRITE) {
            if (!tp_flash_programming_window) {
                // De-energized window closed: the previous control pass left
                // an energized vector latched on the bridge. Programming now
                // would freeze that vector for the whole stall (~0.5 ms with
                // IRQs off) and yank the rotor - retry on the next tick.
                return false;
            }
            uint32_t blk[BLOCK_WORDS];
            uint32_t remaining = RECORD_WORDS - f_word_;
            uint32_t n = (remaining < BLOCK_WORDS) ? remaining : BLOCK_WORDS;
            for (uint32_t i = 0; i < n; ++i) {
                blk[i] = stream_word(f_word_ + i);
            }
            for (uint32_t i = n; i < BLOCK_WORDS; ++i) {
                blk[i] = ERASED_WORD32;  // Pad the tail block with erase state
            }

            if (!flash_->write(f_free_, blk)) {
                f_mode_   = FMODE_IDLE;
                store_ok_ = false;
                TP_LOG("MOTOR: system_dump write fault");
                return true;
            }
            f_free_ += BLOCK_WORDS * 4;
            f_word_ += BLOCK_WORDS;

            if (f_word_ >= RECORD_WORDS) {
                f_mode_   = FMODE_IDLE;
                store_ok_ = true;
                report(false);
            }
            return f_mode_ == FMODE_IDLE;
        }

        return true;  // FMODE_IDLE
    }

    /**
     * @brief Service helper for the system tick: advances whichever engine is
     *        armed by at most ONE 8-word flash block per call.
     * @return true when both engines are idle/done.
     */
    bool service_tick() {
        switch (f_mode_) {
            case FMODE_LOAD:  return load_tick();
            case FMODE_SPACE:
            case FMODE_WRITE: return save_tick();
            default:          return true;
        }
    }
};
