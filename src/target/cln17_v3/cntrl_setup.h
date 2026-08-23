// /**
//  * @file cntrl_setup.h
//  * @brief Control connector runtime configuration and mode switching
//  * 
//  * This module provides a comprehensive interface for managing the control connector
//  * on the CLN17 V3 board. The connector is a multi-function interface where multiple
//  * peripherals (SPI, USART, I2C, encoder, step-dir) share the same physical pins.
//  * 
//  * Key features:
//  * - Runtime mode switching between different peripheral configurations
//  * - Power control for 5V connector supply (CTRL_5V)
//  * - Automatic pin reconfiguration when switching modes
//  * - Safe state management to prevent conflicts (PB_5/PB_7 never active simultaneously)
//  * - Support for SPI, USART, I2C, encoder (ABZ/pulse-dir), step-dir, and GPIO modes
//  * - DMA support for high-performance SPI and USART transfers
//  * - Runtime peripheral reconfiguration (frequency, baud rate, etc.)
//  * - Encoder velocity calculation (counts/sec and RPM)
//  * - Quick setup methods for common configurations
//  * - Blocking and non-blocking operation modes
//  * 
//  * Pin mapping:
//  * - PB_3 (CTRL_SCK):  ERR pin | SPI3_CLK | USART2_TX | TIM3_CH1 (ABZ_Z)
//  * - PB_4 (CTRL_MISO): STP pin | SPI3_MISO | USART2_RX | TIM3_CH2 (ABZ_B)
//  * - PB_5 (CTRL_MOSI): DIR pin | SPI3_MOSI | USART2_CK | TIM3_CH3 (ABZ_A) [Conflicts with PB_7]
//  * - PA_15 (CTRL_CS):  ENB pin | SPI3_NSS | I2C1_SCL
//  * - PB_7 (CTRL_SDA):  DIR pin | I2C1_SDA [Conflicts with PB_5]
//  * - PC_15 (CTRL_5V):  5V power supply enable
//  * 
//  * Safety constraint: PB_5 and PB_7 are electrically connected (both serve as DIR pin)
//  * and must never be configured as outputs simultaneously to prevent hardware damage.
//  * 
//  * Quick Start:
//  * @code
//  * ControlConnector::ControlSetup ctrl;
//  * ctrl.init();
//  * ctrl.set_power(true);
//  * 
//  * // SPI at 5MHz with DMA
//  * ctrl.setup_spi_quick(5.0f, 0, true);
//  * 
//  * // Or USART at 921600 baud with DMA
//  * ctrl.setup_usart_quick(921600, true);
//  * 
//  * // Or I2C at 400kHz
//  * ctrl.setup_i2c_quick(400);
//  * 
//  * // Or encoder with velocity
//  * ctrl.setup_encoder_quick(2048, true, 100);  // 2048 PPR, 100Hz sampling
//  * @endcode
//  * 
//  * See detailed usage examples at the end of this file.
//  * 
//  * Copyright 2026, creapunk
//  * Licensed under the Apache License, Version 2.0
//  */

// #ifndef CNTRL_SETUP_H
// #define CNTRL_SETUP_H

// #include <Arduino.h>
// #include "stm32g4xx.h"
// #include "../cln17_v3/target.h"

// namespace ControlConnector {

// // ==============================================================================
// // OPERATING MODES AND CONFIGURATION
// // ==============================================================================

// /// Operating modes for the control connector
// /// Each mode configures the pins for specific peripheral or interface
// enum class Mode : uint8_t {
//     DISABLED = 0,           // All pins in high-impedance input mode, power off
//     SPI_MASTER,             // SPI3 master mode (SCK, MISO, MOSI, CS)
//     USART_FULL,             // USART2 with TX, RX, CK (full synchronous mode)
//     USART_BASIC,            // USART2 with TX, RX only (asynchronous mode)
//     I2C_MASTER,             // I2C1 master mode (SCL on PA_15, SDA on PB_7)
//     I2C_USART,              // I2C1 (SCL, SDA) + USART2 (TX, RX on PB_3, PB_4)
//     STEP_DIR_EN_ERR,        // Step-dir interface: STP, DIR, ENB, ERR pins as GPIO
//     STEP_DIR_EN_PIN,        // Step-dir with ENB: STP, DIR, ENB as GPIO
//     STEP_DIR_EN_USART,      // Step-dir + USART: DIR as GPIO, STP/ERR as USART TX/RX
//     ENCODER_ABZ,            // Encoder ABZ mode: TIM3 channels for A, B, Z inputs
//     ENCODER_PULSE_DIR,      // Encoder pulse-dir mode: TIM3 for pulse counting
//     GPIO_INDIVIDUAL         // Individual GPIO control (manual pin configuration)
// };

// /// SPI configuration structure
// struct SPIConfig {
//     uint32_t baudrate_prescaler;    // SPI_CR1_BR: 0=/2, 1=/4, 2=/8, 3=/16, 4=/32, 5=/64, 6=/128, 7=/256
//     bool cpol;                       // Clock polarity: false=idle low, true=idle high
//     bool cpha;                       // Clock phase: false=first edge, true=second edge
//     uint8_t datasize;                // Data size in bits (8 or 16)
//     bool use_dma;                    // Enable DMA for transfers
//     uint16_t dma_buffer_size;        // DMA buffer size (if DMA enabled)
// };

// /// USART configuration structure
// struct USARTConfig {
//     uint32_t baudrate;               // Baud rate (e.g., 115200)
//     bool parity_enable;              // Enable parity checking
//     uint8_t parity_type;             // 0=even, 1=odd
//     uint8_t stop_bits;               // 0=1 bit, 2=2 bits
//     bool use_dma_tx;                 // Enable DMA for transmission
//     bool use_dma_rx;                 // Enable DMA for reception
//     uint16_t dma_buffer_size;        // DMA buffer size (if DMA enabled)
// };

// /// I2C configuration structure
// struct I2CConfig {
//     uint32_t timing;                 // I2C timing register value
//     uint32_t speed_hz;               // Bus speed: 100000 (standard), 400000 (fast), 1000000 (fast+)
//     bool use_dma;                    // Enable DMA for transfers
// };

// /// Encoder configuration structure
// struct EncoderConfig {
//     uint32_t pulses_per_revolution;  // Number of encoder pulses per full rotation
//     bool invert_direction;           // Invert counting direction
//     bool enable_velocity;            // Enable velocity calculation
//     uint32_t velocity_sample_us;     // Velocity sampling period in microseconds
// };

// // ==============================================================================
// // PIN STATES AND CONFIGURATION
// // ==============================================================================

// /// GPIO configuration for individual pin control
// struct PinConfig {
//     bool is_output;         // true = output, false = input
//     bool is_opendrain;      // true = open-drain, false = push-pull (for outputs)
//     bool pullup;            // true = pull-up enabled (for inputs)
//     bool pulldown;          // true = pull-down enabled (for inputs)
//     bool initial_state;     // Initial output state (high/low)
// };

// // ==============================================================================
// // CONTROL CONNECTOR CLASS
// // ==============================================================================

// /// Control connector configuration and runtime management
// /// Provides safe, runtime-switchable peripheral mode selection
// struct ControlSetup {
// private:
//     Mode current_mode_;                 // Currently active mode
//     bool power_enabled_;                // 5V power supply state
//     volatile bool mode_switching_;      // Flag to prevent concurrent mode changes
    
//     // Pin state tracking for safety
//     bool pb5_active_;                   // Track if PB_5 is configured as output
//     bool pb7_active_;                   // Track if PB_7 is configured as output
    
//     // Configuration storage for runtime reconfiguration
//     SPIConfig spi_config_;              // Current SPI configuration
//     USARTConfig usart_config_;          // Current USART configuration
//     I2CConfig i2c_config_;              // Current I2C configuration
//     EncoderConfig encoder_config_;      // Current encoder configuration
    
//     // DMA buffers (static allocation)
//     static constexpr uint16_t MAX_DMA_BUFFER = 256;
//     uint8_t spi_tx_buffer_[MAX_DMA_BUFFER] __attribute__((aligned(4)));
//     uint8_t spi_rx_buffer_[MAX_DMA_BUFFER] __attribute__((aligned(4)));
//     uint8_t usart_tx_buffer_[MAX_DMA_BUFFER] __attribute__((aligned(4)));
//     uint8_t usart_rx_buffer_[MAX_DMA_BUFFER] __attribute__((aligned(4)));
    
//     // Encoder velocity tracking
//     volatile uint32_t last_encoder_count_;      // Previous encoder position
//     volatile uint32_t last_encoder_time_us_;    // Timestamp of last velocity calculation
//     volatile int32_t encoder_velocity_cps_;     // Encoder velocity in counts per second
//     volatile uint32_t last_pulse_time_us_;      // Time of last encoder pulse (for period measurement)
//     volatile uint32_t pulse_period_us_;         // Period between pulses in microseconds

// public:
//     /// Constructor - initializes to disabled state with default configurations
//     ControlSetup() 
//         : current_mode_(Mode::DISABLED),
//           power_enabled_(false),
//           mode_switching_(false),
//           pb5_active_(false),
//           pb7_active_(false),
//           last_encoder_count_(0),
//           last_encoder_time_us_(0),
//           encoder_velocity_cps_(0),
//           last_pulse_time_us_(0),
//           pulse_period_us_(0) {
//         // Initialize default SPI configuration
//         spi_config_.baudrate_prescaler = 4;     // /32 divider (default ~5MHz @ 170MHz)
//         spi_config_.cpol = false;
//         spi_config_.cpha = false;
//         spi_config_.datasize = 8;
//         spi_config_.use_dma = false;
//         spi_config_.dma_buffer_size = 64;
        
//         // Initialize default USART configuration
//         usart_config_.baudrate = 115200;
//         usart_config_.parity_enable = false;
//         usart_config_.parity_type = 0;
//         usart_config_.stop_bits = 0;
//         usart_config_.use_dma_tx = false;
//         usart_config_.use_dma_rx = false;
//         usart_config_.dma_buffer_size = 128;
        
//         // Initialize default I2C configuration (100kHz standard mode)
//         i2c_config_.timing = 0x10909CEC;        // 100kHz @ 170MHz
//         i2c_config_.speed_hz = 100000;
//         i2c_config_.use_dma = false;
        
//         // Initialize default encoder configuration
//         encoder_config_.pulses_per_revolution = 1024;
//         encoder_config_.invert_direction = false;
//         encoder_config_.enable_velocity = true;
//         encoder_config_.velocity_sample_us = 10000;  // 10ms sampling
//     }

//     /// Initialize the control connector subsystem
//     /// Sets all pins to safe high-impedance state and disables power
//     void init() {
//         // Enable GPIO clocks
//         RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN | RCC_AHB2ENR_GPIOCEN;
        
//         // Configure PC_15 (CTRL_5V) as output for power control
//         configure_power_pin();
        
//         // Set all control pins to safe high-impedance state
//         set_mode_disabled();
        
//         current_mode_ = Mode::DISABLED;
//     }
    
//     /// Enable or disable 5V power supply on the connector
//     /// @param enable true to enable 5V output, false to disable
//     void set_power(bool enable) {
//         if (enable) {
//             GPIOC->BSRR = GPIO_PIN_15;  // Set PC_15 high to enable 5V
//         } else {
//             GPIOC->BRR = GPIO_PIN_15;   // Set PC_15 low to disable 5V
//         }
//         power_enabled_ = enable;
//     }
    
//     /// Get current power state
//     /// @return true if 5V power is enabled
//     bool is_power_enabled() const {
//         return power_enabled_;
//     }
    
//     /// Get current operating mode
//     /// @return Current mode
//     Mode get_mode() const {
//         return current_mode_;
//     }
    
//     // ==============================================================================
//     // RUNTIME CONFIGURATION METHODS
//     // ==============================================================================
    
//     /// Configure SPI parameters (can be called before or after switching to SPI mode)
//     /// @param config SPI configuration structure
//     /// @return true if configuration was applied successfully
//     bool configure_spi(const SPIConfig& config) {
//         spi_config_ = config;  // Store configuration
        
//         // If already in SPI mode, reconfigure the peripheral
//         if (current_mode_ == Mode::SPI_MASTER) {
//             return reconfigure_spi();
//         }
//         return true;
//     }
    
//     /// Configure USART parameters (can be called before or after switching to USART mode)
//     /// @param config USART configuration structure
//     /// @return true if configuration was applied successfully
//     bool configure_usart(const USARTConfig& config) {
//         usart_config_ = config;  // Store configuration
        
//         // If already in USART mode, reconfigure the peripheral
//         if (current_mode_ == Mode::USART_FULL || current_mode_ == Mode::USART_BASIC ||
//             current_mode_ == Mode::I2C_USART || current_mode_ == Mode::STEP_DIR_EN_USART) {
//             return reconfigure_usart();
//         }
//         return true;
//     }
    
//     /// Configure I2C parameters (can be called before or after switching to I2C mode)
//     /// @param config I2C configuration structure
//     /// @return true if configuration was applied successfully
//     bool configure_i2c(const I2CConfig& config) {
//         i2c_config_ = config;  // Store configuration
        
//         // If already in I2C mode, reconfigure the peripheral
//         if (current_mode_ == Mode::I2C_MASTER || current_mode_ == Mode::I2C_USART) {
//             return reconfigure_i2c();
//         }
//         return true;
//     }
    
//     /// Configure encoder parameters (can be called before or after switching to encoder mode)
//     /// @param config Encoder configuration structure
//     /// @return true if configuration was applied successfully
//     bool configure_encoder(const EncoderConfig& config) {
//         encoder_config_ = config;  // Store configuration
        
//         // Reset velocity tracking when configuration changes
//         last_encoder_count_ = 0;
//         last_encoder_time_us_ = 0;
//         encoder_velocity_cps_ = 0;
//         last_pulse_time_us_ = 0;
//         pulse_period_us_ = 0;
        
//         return true;
//     }
    
//     /// Get current SPI configuration
//     /// @return Reference to SPI configuration
//     const SPIConfig& get_spi_config() const { return spi_config_; }
    
//     /// Get current USART configuration
//     /// @return Reference to USART configuration
//     const USARTConfig& get_usart_config() const { return usart_config_; }
    
//     /// Get current I2C configuration
//     /// @return Reference to I2C configuration
//     const I2CConfig& get_i2c_config() const { return i2c_config_; }
    
//     /// Get current encoder configuration
//     /// @return Reference to encoder configuration
//     const EncoderConfig& get_encoder_config() const { return encoder_config_; }
    
//     /// Switch to a new operating mode
//     /// Safely disables current mode before configuring new mode
//     /// @param new_mode Target operating mode
//     /// @return true if mode switch was successful
//     bool switch_mode(Mode new_mode) {
//         // Prevent concurrent mode switching
//         if (mode_switching_) {
//             return false;
//         }
//         mode_switching_ = true;
        
//         // Disable current configuration
//         set_mode_disabled();
        
//         // Short delay for pin stabilization
//         delayMicroseconds(10);
        
//         // Configure new mode
//         bool success = false;
//         switch (new_mode) {
//             case Mode::DISABLED:
//                 set_mode_disabled();
//                 success = true;
//                 break;
//             case Mode::SPI_MASTER:
//                 success = set_mode_spi_master();
//                 break;
//             case Mode::USART_FULL:
//                 success = set_mode_usart_full();
//                 break;
//             case Mode::USART_BASIC:
//                 success = set_mode_usart_basic();
//                 break;
//             case Mode::I2C_MASTER:
//                 success = set_mode_i2c_master();
//                 break;
//             case Mode::I2C_USART:
//                 success = set_mode_i2c_usart();
//                 break;
//             case Mode::STEP_DIR_EN_ERR:
//                 success = set_mode_step_dir_en_err();
//                 break;
//             case Mode::STEP_DIR_EN_PIN:
//                 success = set_mode_step_dir_en_pin();
//                 break;
//             case Mode::STEP_DIR_EN_USART:
//                 success = set_mode_step_dir_en_usart();
//                 break;
//             case Mode::ENCODER_ABZ:
//                 success = set_mode_encoder_abz();
//                 break;
//             case Mode::ENCODER_PULSE_DIR:
//                 success = set_mode_encoder_pulse_dir();
//                 break;
//             case Mode::GPIO_INDIVIDUAL:
//                 success = set_mode_gpio_individual();
//                 break;
//             default:
//                 success = false;
//                 break;
//         }
        
//         if (success) {
//             current_mode_ = new_mode;
//         } else {
//             // If configuration failed, revert to disabled state
//             set_mode_disabled();
//             current_mode_ = Mode::DISABLED;
//         }
        
//         mode_switching_ = false;
//         return success;
//     }

// private:
//     // ==============================================================================
//     // PIN CONFIGURATION HELPERS
//     // ==============================================================================
    
//     /// Configure PC_15 as output for 5V power control
//     void configure_power_pin() {
//         // PC_15 as GPIO output, push-pull, no pull, low speed
//         GPIOC->MODER &= ~(3U << (15 * 2));
//         GPIOC->MODER |= (1U << (15 * 2));           // Output mode
//         GPIOC->OTYPER &= ~GPIO_OTYPER_OT15;         // Push-pull
//         GPIOC->OSPEEDR &= ~(3U << (15 * 2));        // Low speed
//         GPIOC->PUPDR &= ~(3U << (15 * 2));          // No pull
//         GPIOC->BRR = GPIO_PIN_15;                   // Initial state: low (power off)
//     }
    
//     /// Set pin to input mode with optional pull resistor
//     /// @param gpio GPIO port
//     /// @param pin Pin number (0-15)
//     /// @param pull 0=no pull, 1=pull-up, 2=pull-down
//     void set_pin_input(GPIO_TypeDef* gpio, uint8_t pin, uint8_t pull = 0) {
//         gpio->MODER &= ~(3U << (pin * 2));          // Clear mode bits (00 = input)
//         gpio->PUPDR &= ~(3U << (pin * 2));          // Clear pull bits
//         gpio->PUPDR |= (pull << (pin * 2));         // Set pull resistor
        
//         // Track PB_5 and PB_7 state
//         if (gpio == GPIOB && pin == 5) pb5_active_ = false;
//         if (gpio == GPIOB && pin == 7) pb7_active_ = false;
//     }
    
//     /// Set pin to output mode
//     /// @param gpio GPIO port
//     /// @param pin Pin number (0-15)
//     /// @param open_drain true for open-drain, false for push-pull
//     /// @param speed 0=low, 1=medium, 2=fast, 3=high
//     /// @param initial_state Initial output level (true=high, false=low)
//     void set_pin_output(GPIO_TypeDef* gpio, uint8_t pin, bool open_drain = false, 
//                        uint8_t speed = 2, bool initial_state = false) {
//         // Safety check: prevent PB_5 and PB_7 both being outputs
//         if (gpio == GPIOB) {
//             if (pin == 5 && pb7_active_) return;    // PB_7 already active, cannot activate PB_5
//             if (pin == 7 && pb5_active_) return;    // PB_5 already active, cannot activate PB_7
//         }
        
//         // Set initial state before configuring as output
//         if (initial_state) {
//             gpio->BSRR = (1U << pin);               // Set high
//         } else {
//             gpio->BRR = (1U << pin);                // Set low
//         }
        
//         gpio->MODER &= ~(3U << (pin * 2));          // Clear mode bits
//         gpio->MODER |= (1U << (pin * 2));           // Set to output (01)
        
//         if (open_drain) {
//             gpio->OTYPER |= (1U << pin);            // Open-drain
//         } else {
//             gpio->OTYPER &= ~(1U << pin);           // Push-pull
//         }
        
//         gpio->OSPEEDR &= ~(3U << (pin * 2));        // Clear speed bits
//         gpio->OSPEEDR |= (speed << (pin * 2));      // Set speed
        
//         gpio->PUPDR &= ~(3U << (pin * 2));          // No pull resistor for outputs
        
//         // Track PB_5 and PB_7 state
//         if (gpio == GPIOB && pin == 5) pb5_active_ = true;
//         if (gpio == GPIOB && pin == 7) pb7_active_ = true;
//     }
    
//     /// Set pin to alternate function mode
//     /// @param gpio GPIO port
//     /// @param pin Pin number (0-15)
//     /// @param af Alternate function number (0-15)
//     /// @param speed 0=low, 1=medium, 2=fast, 3=high
//     /// @param pull 0=no pull, 1=pull-up, 2=pull-down
//     void set_pin_alternate(GPIO_TypeDef* gpio, uint8_t pin, uint8_t af, 
//                           uint8_t speed = 3, uint8_t pull = 0) {
//         // Safety check: prevent PB_5 and PB_7 both being configured for AF
//         if (gpio == GPIOB) {
//             if (pin == 5 && pb7_active_) return;    // PB_7 already active
//             if (pin == 7 && pb5_active_) return;    // PB_5 already active
//         }
        
//         gpio->MODER &= ~(3U << (pin * 2));          // Clear mode bits
//         gpio->MODER |= (2U << (pin * 2));           // Set to alternate function (10)
        
//         gpio->OTYPER &= ~(1U << pin);               // Push-pull
        
//         gpio->OSPEEDR &= ~(3U << (pin * 2));        // Clear speed bits
//         gpio->OSPEEDR |= (speed << (pin * 2));      // Set speed
        
//         gpio->PUPDR &= ~(3U << (pin * 2));          // Clear pull bits
//         gpio->PUPDR |= (pull << (pin * 2));         // Set pull resistor
        
//         // Set alternate function
//         if (pin < 8) {
//             gpio->AFR[0] &= ~(0xFU << (pin * 4));   // Clear AF bits
//             gpio->AFR[0] |= (af << (pin * 4));      // Set AF
//         } else {
//             gpio->AFR[1] &= ~(0xFU << ((pin - 8) * 4)); // Clear AF bits
//             gpio->AFR[1] |= (af << ((pin - 8) * 4));    // Set AF
//         }
        
//         // Track PB_5 and PB_7 state (AF mode counts as active)
//         if (gpio == GPIOB && pin == 5) pb5_active_ = true;
//         if (gpio == GPIOB && pin == 7) pb7_active_ = true;
//     }
    
//     // ==============================================================================
//     // MODE CONFIGURATION IMPLEMENTATIONS
//     // ==============================================================================
    
//     /// Disable all pins - set to high-impedance input
//     void set_mode_disabled() {
//         // Disable peripherals if active
//         disable_spi3();
//         disable_usart2();
//         disable_i2c1();
//         disable_tim3();
        
//         // Set all control pins to high-impedance input (no pull)
//         set_pin_input(GPIOB, 3, 0);  // PB_3 (CTRL_SCK)
//         set_pin_input(GPIOB, 4, 0);  // PB_4 (CTRL_MISO)
//         set_pin_input(GPIOB, 5, 0);  // PB_5 (CTRL_MOSI)
//         set_pin_input(GPIOA, 15, 0); // PA_15 (CTRL_CS)
//         set_pin_input(GPIOB, 7, 0);  // PB_7 (CTRL_SDA)
        
//         pb5_active_ = false;
//         pb7_active_ = false;
//     }
    
//     /// Configure SPI3 master mode
//     /// Pins: PB_3=SCK, PB_4=MISO, PB_5=MOSI, PA_15=CS
//     bool set_mode_spi_master() {
//         // Enable SPI3 clock
//         RCC->APB1ENR1 |= RCC_APB1ENR1_SPI3EN;
        
//         // Configure pins for SPI3 (AF6)
//         set_pin_alternate(GPIOB, 3, 6, 3, 0);   // PB_3 = SPI3_SCK
//         set_pin_alternate(GPIOB, 4, 6, 3, 0);   // PB_4 = SPI3_MISO
//         set_pin_alternate(GPIOB, 5, 6, 3, 0);   // PB_5 = SPI3_MOSI
//         set_pin_output(GPIOA, 15, false, 2, true); // PA_15 = CS (GPIO, initially high)
        
//         // Apply configuration
//         return reconfigure_spi();
//     }
    
//     /// Configure USART2 full mode (TX, RX, CK)
//     /// Pins: PB_3=TX, PB_4=RX, PB_5=CK
//     bool set_mode_usart_full() {
//         // Enable USART2 clock
//         RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN;
        
//         // Configure pins for USART2 (AF7)
//         set_pin_alternate(GPIOB, 3, 7, 3, 0);   // PB_3 = USART2_TX
//         set_pin_alternate(GPIOB, 4, 7, 3, 1);   // PB_4 = USART2_RX (with pull-up)
//         set_pin_alternate(GPIOB, 5, 7, 3, 0);   // PB_5 = USART2_CK
        
//         // Apply configuration
//         return reconfigure_usart();
//     }
    
//     /// Configure USART2 basic mode (TX, RX only)
//     /// Pins: PB_3=TX, PB_4=RX
//     bool set_mode_usart_basic() {
//         // Enable USART2 clock
//         RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN;
        
//         // Configure pins for USART2 (AF7)
//         set_pin_alternate(GPIOB, 3, 7, 3, 0);   // PB_3 = USART2_TX
//         set_pin_alternate(GPIOB, 4, 7, 3, 1);   // PB_4 = USART2_RX (with pull-up)
//         set_pin_input(GPIOB, 5, 0);             // PB_5 = unused (hi-z)
        
//         // Apply configuration
//         return reconfigure_usart();
//     }
    
//     /// Configure I2C1 master mode
//     /// Pins: PA_15=SCL, PB_7=SDA
//     bool set_mode_i2c_master() {
//         // Enable I2C1 clock
//         RCC->APB1ENR1 |= RCC_APB1ENR1_I2C1EN;
        
//         // Configure pins for I2C1 (AF4)
//         set_pin_alternate(GPIOA, 15, 4, 3, 0);  // PA_15 = I2C1_SCL
//         set_pin_alternate(GPIOB, 7, 4, 3, 0);   // PB_7 = I2C1_SDA
        
//         // Open-drain outputs for I2C
//         GPIOA->OTYPER |= GPIO_OTYPER_OT15;      // SCL open-drain
//         GPIOB->OTYPER |= GPIO_OTYPER_OT7;       // SDA open-drain
        
//         // Apply configuration
//         return reconfigure_i2c();
//     }
    
//     /// Configure I2C1 + USART2 combined mode
//     /// Pins: PA_15=SCL, PB_7=SDA, PB_3=TX, PB_4=RX
//     bool set_mode_i2c_usart() {
//         // Enable clocks
//         RCC->APB1ENR1 |= RCC_APB1ENR1_I2C1EN | RCC_APB1ENR1_USART2EN;
        
//         // Configure I2C pins
//         set_pin_alternate(GPIOA, 15, 4, 3, 0);  // PA_15 = I2C1_SCL
//         set_pin_alternate(GPIOB, 7, 4, 3, 0);   // PB_7 = I2C1_SDA
//         GPIOA->OTYPER |= GPIO_OTYPER_OT15;      // SCL open-drain
//         GPIOB->OTYPER |= GPIO_OTYPER_OT7;       // SDA open-drain
        
//         // Configure USART pins
//         set_pin_alternate(GPIOB, 3, 7, 3, 0);   // PB_3 = USART2_TX
//         set_pin_alternate(GPIOB, 4, 7, 3, 1);   // PB_4 = USART2_RX
        
//         // Initialize both peripherals
//         reconfigure_i2c();
//         reconfigure_usart();
        
//         return true;
//     }
    
//     /// Configure step-dir interface with enable and error pins
//     /// Pins: PB_4=STP(in), PB_5=DIR(in), PA_15=ENB(out), PB_3=ERR(out)
//     bool set_mode_step_dir_en_err() {
//         set_pin_input(GPIOB, 4, 0);             // PB_4 = STEP input
//         set_pin_input(GPIOB, 5, 0);             // PB_5 = DIR input
//         set_pin_output(GPIOA, 15, false, 2, false); // PA_15 = ENABLE output (low=disabled)
//         set_pin_output(GPIOB, 3, false, 2, false);  // PB_3 = ERROR output (low=no error)
//         return true;
//     }
    
//     /// Configure step-dir interface with enable pin only
//     /// Pins: PB_4=STP(in), PB_5=DIR(in), PA_15=ENB(out)
//     bool set_mode_step_dir_en_pin() {
//         set_pin_input(GPIOB, 4, 0);             // PB_4 = STEP input
//         set_pin_input(GPIOB, 5, 0);             // PB_5 = DIR input
//         set_pin_output(GPIOA, 15, false, 2, false); // PA_15 = ENABLE output
//         set_pin_input(GPIOB, 3, 0);             // PB_3 = unused (hi-z)
//         return true;
//     }
    
//     /// Configure step-dir with USART communication
//     /// Pins: PB_5=DIR(in), PB_3=TX, PB_4=RX, PA_15=ENB(out)
//     /// Note: Uses PB_5 for DIR (not PB_7) to avoid conflict
//     bool set_mode_step_dir_en_usart() {
//         // Enable USART2 clock
//         RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN;
        
//         set_pin_input(GPIOB, 5, 0);             // PB_5 = DIR input
//         set_pin_alternate(GPIOB, 3, 7, 3, 0);   // PB_3 = USART2_TX
//         set_pin_alternate(GPIOB, 4, 7, 3, 1);   // PB_4 = USART2_RX
//         set_pin_output(GPIOA, 15, false, 2, false); // PA_15 = ENABLE output
        
//         // Apply configuration
//         return reconfigure_usart();
//     }
    
//     /// Configure encoder ABZ input mode using TIM3
//     /// Pins: PB_5=A(TIM3_CH3), PB_4=B(TIM3_CH2), PB_3=Z(TIM3_CH1)
//     bool set_mode_encoder_abz() {
//         // Enable TIM3 clock
//         RCC->APB1ENR1 |= RCC_APB1ENR1_TIM3EN;
        
//         // Configure pins for TIM3 channels (AF2)
//         set_pin_alternate(GPIOB, 5, 2, 2, 0);   // PB_5 = TIM3_CH3 (A)
//         set_pin_alternate(GPIOB, 4, 2, 2, 0);   // PB_4 = TIM3_CH2 (B)
//         set_pin_alternate(GPIOB, 3, 2, 2, 0);   // PB_3 = TIM3_CH1 (Z/Index)
        
//         // Configure TIM3 for encoder interface mode
//         TIM3->CCMR1 = (1U << TIM_CCMR1_CC1S_Pos) | (1U << TIM_CCMR1_CC2S_Pos); // CH1, CH2 as inputs
//         TIM3->CCMR2 = (1U << TIM_CCMR2_CC3S_Pos);                              // CH3 as input
//         TIM3->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E;           // Enable all channels
//         TIM3->SMCR = (3U << TIM_SMCR_SMS_Pos);  // Encoder mode 3 (count on both edges)
//         TIM3->ARR = 0xFFFF;                     // Max count value
//         TIM3->CNT = 0;                          // Reset counter
//         TIM3->CR1 = TIM_CR1_CEN;                // Enable counter
        
//         return true;
//     }
    
//     /// Configure encoder pulse-direction mode using TIM3
//     /// Pins: PB_4=Pulse(TIM3_CH2), PB_5=Dir(TIM3_CH3)
//     bool set_mode_encoder_pulse_dir() {
//         // Enable TIM3 clock
//         RCC->APB1ENR1 |= RCC_APB1ENR1_TIM3EN;
        
//         // Configure pins for TIM3 channels (AF2)
//         set_pin_alternate(GPIOB, 4, 2, 2, 0);   // PB_4 = TIM3_CH2 (Pulse)
//         set_pin_alternate(GPIOB, 5, 2, 2, 0);   // PB_5 = TIM3_CH3 (Direction)
        
//         // Configure TIM3 for up/down counting based on direction input
//         TIM3->CCMR1 = (1U << TIM_CCMR1_CC2S_Pos); // CH2 as input (pulse)
//         TIM3->CCMR2 = (1U << TIM_CCMR2_CC3S_Pos); // CH3 as input (direction)
//         TIM3->CCER = TIM_CCER_CC2E | TIM_CCER_CC3E; // Enable channels
//         TIM3->SMCR = (1U << TIM_SMCR_SMS_Pos);  // Encoder mode 1 (count on TI2 edges)
//         TIM3->ARR = 0xFFFF;                     // Max count value
//         TIM3->CNT = 0;                          // Reset counter
//         TIM3->CR1 = TIM_CR1_CEN;                // Enable counter
        
//         return true;
//     }
    
//     /// Configure individual GPIO control mode
//     /// All pins set to inputs, user must configure individually
//     bool set_mode_gpio_individual() {
//         set_pin_input(GPIOB, 3, 0);  // PB_3 as input
//         set_pin_input(GPIOB, 4, 0);  // PB_4 as input
//         set_pin_input(GPIOB, 5, 0);  // PB_5 as input
//         set_pin_input(GPIOA, 15, 0); // PA_15 as input
//         set_pin_input(GPIOB, 7, 0);  // PB_7 as input
//         return true;
//     }
    
//     // ==============================================================================
//     // RUNTIME RECONFIGURATION HELPERS
//     // ==============================================================================
    
//     /// Reconfigure SPI peripheral with current settings
//     bool reconfigure_spi() {
//         if (current_mode_ != Mode::SPI_MASTER) {
//             return false;
//         }
        
//         // Disable SPI
//         SPI3->CR1 &= ~SPI_CR1_SPE;
        
//         // Stop DMA if active
//         if (spi_config_.use_dma) {
//             stop_spi_dma();
//         }
        
//         // Configure SPI CR1 register
//         uint32_t cr1 = SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI;  // Master mode, software CS
//         cr1 |= (spi_config_.baudrate_prescaler & 0x07) << SPI_CR1_BR_Pos;  // Baud rate
        
//         if (spi_config_.cpol) {
//             cr1 |= SPI_CR1_CPOL;  // Clock polarity
//         }
//         if (spi_config_.cpha) {
//             cr1 |= SPI_CR1_CPHA;  // Clock phase
//         }
        
//         SPI3->CR1 = cr1;
        
//         // Configure SPI CR2 register
//         uint32_t cr2 = SPI_CR2_FRXTH;  // RXNE at 8-bit threshold
//         cr2 |= ((spi_config_.datasize - 1) & 0x0F) << SPI_CR2_DS_Pos;  // Data size
        
//         if (spi_config_.use_dma) {
//             cr2 |= SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN;  // Enable DMA
//             init_spi_dma();  // Initialize DMA channels
//         }
        
//         SPI3->CR2 = cr2;
        
//         // Enable SPI
//         SPI3->CR1 |= SPI_CR1_SPE;
        
//         return true;
//     }
    
//     /// Reconfigure USART peripheral with current settings
//     bool reconfigure_usart() {
//         // Disable USART
//         USART2->CR1 &= ~USART_CR1_UE;
        
//         // Stop DMA if active
//         if (usart_config_.use_dma_tx || usart_config_.use_dma_rx) {
//             stop_usart_dma();
//         }
        
//         // Calculate and set baud rate
//         USART2->BRR = SystemCoreClock / usart_config_.baudrate;
        
//         // Configure CR1 register
//         uint32_t cr1 = USART_CR1_TE | USART_CR1_RE;  // Enable TX and RX
        
//         if (usart_config_.parity_enable) {
//             cr1 |= USART_CR1_PCE;  // Enable parity
//             if (usart_config_.parity_type == 1) {
//                 cr1 |= USART_CR1_PS;  // Odd parity
//             }
//         }
        
//         USART2->CR1 = cr1;
        
//         // Configure CR2 register
//         uint32_t cr2 = 0;
//         if (usart_config_.stop_bits == 2) {
//             cr2 |= (2U << USART_CR2_STOP_Pos);  // 2 stop bits
//         }
        
//         if (current_mode_ == Mode::USART_FULL) {
//             cr2 |= USART_CR2_CLKEN;  // Enable clock output for synchronous mode
//         }
        
//         USART2->CR2 = cr2;
        
//         // Configure CR3 register for DMA
//         uint32_t cr3 = 0;
//         if (usart_config_.use_dma_tx) {
//             cr3 |= USART_CR3_DMAT;  // Enable DMA for transmission
//             init_usart_tx_dma();
//         }
//         if (usart_config_.use_dma_rx) {
//             cr3 |= USART_CR3_DMAR;  // Enable DMA for reception
//             init_usart_rx_dma();
//         }
        
//         USART2->CR3 = cr3;
        
//         // Enable USART
//         USART2->CR1 |= USART_CR1_UE;
        
//         return true;
//     }
    
//     /// Reconfigure I2C peripheral with current settings
//     bool reconfigure_i2c() {
//         // Disable I2C
//         I2C1->CR1 &= ~I2C_CR1_PE;
        
//         // Calculate timing based on speed
//         uint32_t timing;
//         if (i2c_config_.speed_hz <= 100000) {
//             // Standard mode 100kHz
//             timing = 0x10909CEC;
//         } else if (i2c_config_.speed_hz <= 400000) {
//             // Fast mode 400kHz
//             timing = 0x00702991;
//         } else {
//             // Fast mode plus 1MHz
//             timing = 0x00300209;
//         }
        
//         i2c_config_.timing = timing;
//         I2C1->TIMINGR = timing;
        
//         // Enable I2C
//         I2C1->CR1 = I2C_CR1_PE;
        
//         return true;
//     }
    
//     // ==============================================================================
//     // DMA INITIALIZATION AND MANAGEMENT
//     // ==============================================================================
    
//     /// Initialize SPI DMA channels
//     void init_spi_dma() {
//         // Enable DMA1 clock
//         RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN | RCC_AHB1ENR_DMAMUX1EN;
        
//         // Configure DMAMUX for SPI3
//         DMAMUX1_Channel3->CCR = 61U;  // SPI3_RX
//         DMAMUX1_Channel4->CCR = 62U;  // SPI3_TX
        
//         // Configure DMA1 Channel 3 for SPI3_RX
//         DMA1_Channel3->CCR &= ~DMA_CCR_EN;  // Disable channel
//         DMA1_Channel3->CPAR = (uint32_t)&SPI3->DR;  // Peripheral address
//         DMA1_Channel3->CMAR = (uint32_t)spi_rx_buffer_;  // Memory address
//         DMA1_Channel3->CCR = DMA_CCR_MINC | DMA_CCR_TCIE |  // Memory increment, transfer complete interrupt
//                              (3U << DMA_CCR_PL_Pos);  // Very high priority
        
//         // Configure DMA1 Channel 4 for SPI3_TX
//         DMA1_Channel4->CCR &= ~DMA_CCR_EN;  // Disable channel
//         DMA1_Channel4->CPAR = (uint32_t)&SPI3->DR;  // Peripheral address
//         DMA1_Channel4->CMAR = (uint32_t)spi_tx_buffer_;  // Memory address
//         DMA1_Channel4->CCR = DMA_CCR_MINC | DMA_CCR_DIR |  // Memory increment, read from memory
//                              (2U << DMA_CCR_PL_Pos);  // High priority
//     }
    
//     /// Start SPI DMA transfer
//     bool start_spi_dma_transfer(uint16_t length) {
//         if (length == 0 || length > MAX_DMA_BUFFER) {
//             return false;
//         }
        
//         // Clear all flags
//         DMA1->IFCR = DMA_IFCR_CGIF3 | DMA_IFCR_CGIF4;
        
//         // Configure transfer length
//         DMA1_Channel3->CNDTR = length;  // RX
//         DMA1_Channel4->CNDTR = length;  // TX
        
//         // Enable channels
//         DMA1_Channel3->CCR |= DMA_CCR_EN;  // Enable RX first
//         DMA1_Channel4->CCR |= DMA_CCR_EN;  // Enable TX
        
//         return true;
//     }
    
//     /// Stop SPI DMA
//     void stop_spi_dma() {
//         DMA1_Channel3->CCR &= ~DMA_CCR_EN;  // Disable RX
//         DMA1_Channel4->CCR &= ~DMA_CCR_EN;  // Disable TX
//         DMA1->IFCR = DMA_IFCR_CGIF3 | DMA_IFCR_CGIF4;  // Clear flags
//     }
    
//     /// Initialize USART TX DMA
//     void init_usart_tx_dma() {
//         // Enable DMA1 clock
//         RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN | RCC_AHB1ENR_DMAMUX1EN;
        
//         // Configure DMAMUX for USART2_TX
//         DMAMUX1_Channel7->CCR = 57U;  // USART2_TX
        
//         // Configure DMA1 Channel 7 for USART2_TX
//         DMA1_Channel7->CCR &= ~DMA_CCR_EN;  // Disable channel
//         DMA1_Channel7->CPAR = (uint32_t)&USART2->TDR;  // Peripheral address
//         DMA1_Channel7->CMAR = (uint32_t)usart_tx_buffer_;  // Memory address
//         DMA1_Channel7->CCR = DMA_CCR_MINC | DMA_CCR_DIR | DMA_CCR_TCIE |  // Memory increment, read from memory, TC interrupt
//                              (2U << DMA_CCR_PL_Pos);  // High priority
//     }
    
//     /// Initialize USART RX DMA
//     void init_usart_rx_dma() {
//         // Enable DMA1 clock
//         RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN | RCC_AHB1ENR_DMAMUX1EN;
        
//         // Configure DMAMUX for USART2_RX
//         DMAMUX1_Channel6->CCR = 56U;  // USART2_RX
        
//         // Configure DMA1 Channel 6 for USART2_RX
//         DMA1_Channel6->CCR &= ~DMA_CCR_EN;  // Disable channel
//         DMA1_Channel6->CPAR = (uint32_t)&USART2->RDR;  // Peripheral address
//         DMA1_Channel6->CMAR = (uint32_t)usart_rx_buffer_;  // Memory address
//         DMA1_Channel6->CCR = DMA_CCR_MINC | DMA_CCR_TCIE |  // Memory increment, TC interrupt
//                              (3U << DMA_CCR_PL_Pos);  // Very high priority
//     }
    
//     /// Start USART TX DMA
//     bool start_usart_tx_dma(uint16_t length) {
//         if (length == 0 || length > MAX_DMA_BUFFER) {
//             return false;
//         }
        
//         // Clear flags
//         DMA1->IFCR = DMA_IFCR_CGIF7;
        
//         // Configure transfer length
//         DMA1_Channel7->CNDTR = length;
        
//         // Enable channel
//         DMA1_Channel7->CCR |= DMA_CCR_EN;
        
//         return true;
//     }
    
//     /// Start USART RX DMA
//     bool start_usart_rx_dma(uint16_t length) {
//         if (length == 0 || length > MAX_DMA_BUFFER) {
//             return false;
//         }
        
//         // Clear flags
//         DMA1->IFCR = DMA_IFCR_CGIF6;
        
//         // Configure transfer length
//         DMA1_Channel6->CNDTR = length;
        
//         // Enable channel
//         DMA1_Channel6->CCR |= DMA_CCR_EN;
        
//         return true;
//     }
    
//     /// Stop USART DMA
//     void stop_usart_dma() {
//         DMA1_Channel6->CCR &= ~DMA_CCR_EN;  // Disable RX
//         DMA1_Channel7->CCR &= ~DMA_CCR_EN;  // Disable TX
//         DMA1->IFCR = DMA_IFCR_CGIF6 | DMA_IFCR_CGIF7;  // Clear flags
//     }
    
//     // ==============================================================================
//     // PERIPHERAL DISABLE FUNCTIONS
//     // ==============================================================================
    
//     /// Disable SPI3 peripheral
//     void disable_spi3() {
//         if (RCC->APB1ENR1 & RCC_APB1ENR1_SPI3EN) {
//             SPI3->CR1 &= ~SPI_CR1_SPE;          // Disable SPI
//             if (spi_config_.use_dma) {
//                 stop_spi_dma();                  // Stop DMA if enabled
//             }
//             RCC->APB1ENR1 &= ~RCC_APB1ENR1_SPI3EN; // Disable clock
//         }
//     }
    
//     /// Disable USART2 peripheral
//     void disable_usart2() {
//         if (RCC->APB1ENR1 & RCC_APB1ENR1_USART2EN) {
//             USART2->CR1 &= ~USART_CR1_UE;       // Disable USART
//             if (usart_config_.use_dma_tx || usart_config_.use_dma_rx) {
//                 stop_usart_dma();                // Stop DMA if enabled
//             }
//             RCC->APB1ENR1 &= ~RCC_APB1ENR1_USART2EN; // Disable clock
//         }
//     }
    
//     /// Disable I2C1 peripheral
//     void disable_i2c1() {
//         if (RCC->APB1ENR1 & RCC_APB1ENR1_I2C1EN) {
//             I2C1->CR1 &= ~I2C_CR1_PE;           // Disable I2C
//             RCC->APB1ENR1 &= ~RCC_APB1ENR1_I2C1EN; // Disable clock
//         }
//     }
    
//     /// Disable TIM3 peripheral
//     void disable_tim3() {
//         if (RCC->APB1ENR1 & RCC_APB1ENR1_TIM3EN) {
//             TIM3->CR1 &= ~TIM_CR1_CEN;          // Disable timer
//             RCC->APB1ENR1 &= ~RCC_APB1ENR1_TIM3EN; // Disable clock
//         }
//     }

// public:
//     // ==============================================================================
//     // INDIVIDUAL PIN CONTROL (for GPIO_INDIVIDUAL mode)
//     // ==============================================================================
    
//     /// Configure individual pin (only in GPIO_INDIVIDUAL mode)
//     /// @param pin Pin identifier (3, 4, 5, 15, 7 for respective pins)
//     /// @param config Pin configuration structure
//     /// @return true if successful
//     bool configure_pin(uint8_t pin, const PinConfig& config) {
//         if (current_mode_ != Mode::GPIO_INDIVIDUAL) {
//             return false; // Only allowed in GPIO_INDIVIDUAL mode
//         }
        
//         GPIO_TypeDef* gpio;
//         uint8_t pin_num;
        
//         // Map pin identifiers to GPIO port and pin number
//         switch (pin) {
//             case 3:  gpio = GPIOB; pin_num = 3; break;   // PB_3
//             case 4:  gpio = GPIOB; pin_num = 4; break;   // PB_4
//             case 5:  gpio = GPIOB; pin_num = 5; break;   // PB_5
//             case 15: gpio = GPIOA; pin_num = 15; break;  // PA_15
//             case 7:  gpio = GPIOB; pin_num = 7; break;   // PB_7
//             default: return false; // Invalid pin
//         }
        
//         if (config.is_output) {
//             set_pin_output(gpio, pin_num, config.is_opendrain, 2, config.initial_state);
//         } else {
//             uint8_t pull = 0;
//             if (config.pullup) pull = 1;
//             else if (config.pulldown) pull = 2;
//             set_pin_input(gpio, pin_num, pull);
//         }
        
//         return true;
//     }
    
//     /// Read digital state of a pin
//     /// @param pin Pin identifier
//     /// @return true if high, false if low
//     bool read_pin(uint8_t pin) const {
//         GPIO_TypeDef* gpio;
//         uint8_t pin_num;
        
//         switch (pin) {
//             case 3:  gpio = GPIOB; pin_num = 3; break;
//             case 4:  gpio = GPIOB; pin_num = 4; break;
//             case 5:  gpio = GPIOB; pin_num = 5; break;
//             case 15: gpio = GPIOA; pin_num = 15; break;
//             case 7:  gpio = GPIOB; pin_num = 7; break;
//             default: return false;
//         }
        
//         return (gpio->IDR & (1U << pin_num)) != 0;
//     }
    
//     /// Write digital state to a pin (must be configured as output)
//     /// @param pin Pin identifier
//     /// @param state true for high, false for low
//     void write_pin(uint8_t pin, bool state) {
//         GPIO_TypeDef* gpio;
//         uint8_t pin_num;
        
//         switch (pin) {
//             case 3:  gpio = GPIOB; pin_num = 3; break;
//             case 4:  gpio = GPIOB; pin_num = 4; break;
//             case 5:  gpio = GPIOB; pin_num = 5; break;
//             case 15: gpio = GPIOA; pin_num = 15; break;
//             case 7:  gpio = GPIOB; pin_num = 7; break;
//             default: return;
//         }
        
//         if (state) {
//             gpio->BSRR = (1U << pin_num);   // Set high
//         } else {
//             gpio->BRR = (1U << pin_num);    // Set low
//         }
//     }
    
//     // ==============================================================================
//     // CONVENIENCE METHODS FOR EASY CONFIGURATION
//     // ==============================================================================
    
//     /// Quick setup for SPI mode with common parameters
//     /// @param speed_mhz Approximate SPI speed in MHz (actual speed depends on prescaler)
//     /// @param mode SPI mode: 0 (CPOL=0,CPHA=0), 1 (0,1), 2 (1,0), 3 (1,1)
//     /// @param use_dma Enable DMA transfers
//     /// @return true if mode switched successfully
//     bool setup_spi_quick(float speed_mhz, uint8_t mode = 0, bool use_dma = false) {
//         // Calculate prescaler for desired speed (SystemClock = 170MHz)
//         uint32_t target_freq = (uint32_t)(speed_mhz * 1000000);
//         uint32_t prescaler = 0;
        
//         // Find appropriate prescaler: /2, /4, /8, /16, /32, /64, /128, /256
//         uint32_t freq_test[] = {85000000, 42500000, 21250000, 10625000, 5312500, 2656250, 1328125, 664062};
//         for (uint8_t i = 0; i < 8; i++) {
//             if (target_freq >= freq_test[i]) {
//                 prescaler = i;
//                 break;
//             }
//         }
        
//         SPIConfig cfg;
//         cfg.baudrate_prescaler = prescaler;
//         cfg.cpol = (mode & 0x02) != 0;
//         cfg.cpha = (mode & 0x01) != 0;
//         cfg.datasize = 8;
//         cfg.use_dma = use_dma;
//         cfg.dma_buffer_size = 64;
        
//         configure_spi(cfg);
//         return switch_mode(Mode::SPI_MASTER);
//     }
    
//     /// Quick setup for USART mode with common parameters
//     /// @param baudrate Desired baud rate (e.g., 115200, 921600)
//     /// @param use_dma Enable DMA for TX and RX
//     /// @return true if mode switched successfully
//     bool setup_usart_quick(uint32_t baudrate, bool use_dma = false) {
//         USARTConfig cfg;
//         cfg.baudrate = baudrate;
//         cfg.parity_enable = false;
//         cfg.parity_type = 0;
//         cfg.stop_bits = 0;
//         cfg.use_dma_tx = use_dma;
//         cfg.use_dma_rx = use_dma;
//         cfg.dma_buffer_size = 128;
        
//         configure_usart(cfg);
//         return switch_mode(Mode::USART_BASIC);
//     }
    
//     /// Quick setup for I2C mode with common parameters
//     /// @param speed_khz I2C speed in kHz: 100 (standard), 400 (fast), 1000 (fast+)
//     /// @return true if mode switched successfully
//     bool setup_i2c_quick(uint32_t speed_khz) {
//         I2CConfig cfg;
//         cfg.speed_hz = speed_khz * 1000;
//         cfg.use_dma = false;
        
//         // Calculate timing based on speed
//         if (speed_khz <= 100) {
//             cfg.timing = 0x10909CEC;  // 100kHz
//         } else if (speed_khz <= 400) {
//             cfg.timing = 0x00702991;  // 400kHz
//         } else {
//             cfg.timing = 0x00300209;  // 1MHz
//         }
        
//         configure_i2c(cfg);
//         return switch_mode(Mode::I2C_MASTER);
//     }
    
//     /// Quick setup for encoder mode with common parameters
//     /// @param ppr Pulses per revolution
//     /// @param enable_velocity Enable velocity calculation
//     /// @param sample_rate_hz Velocity sampling rate in Hz (e.g., 100 for 100Hz)
//     /// @return true if mode switched successfully
//     bool setup_encoder_quick(uint32_t ppr, bool enable_velocity = true, uint32_t sample_rate_hz = 100) {
//         EncoderConfig cfg;
//         cfg.pulses_per_revolution = ppr;
//         cfg.invert_direction = false;
//         cfg.enable_velocity = enable_velocity;
//         cfg.velocity_sample_us = 1000000 / sample_rate_hz;
        
//         configure_encoder(cfg);
//         return switch_mode(Mode::ENCODER_ABZ);
//     }
    
//     /// Blocking SPI transfer (non-DMA, polling mode)
//     /// @param tx_data Transmit buffer (can be nullptr for RX-only)
//     /// @param rx_data Receive buffer (can be nullptr for TX-only)
//     /// @param length Number of bytes to transfer
//     /// @param timeout_ms Timeout in milliseconds
//     /// @return true if transfer completed successfully
//     bool spi_transfer_blocking(const uint8_t* tx_data, uint8_t* rx_data, uint16_t length, uint32_t timeout_ms = 1000) {
//         if (current_mode_ != Mode::SPI_MASTER) {
//             return false;
//         }
        
//         uint32_t start_time = millis();
        
//         for (uint16_t i = 0; i < length; i++) {
//             // Wait for TXE flag
//             while (!(SPI3->SR & SPI_SR_TXE)) {
//                 if ((millis() - start_time) > timeout_ms) return false;
//             }
            
//             // Send byte
//             *(volatile uint8_t*)&SPI3->DR = tx_data ? tx_data[i] : 0xFF;
            
//             // Wait for RXNE flag
//             while (!(SPI3->SR & SPI_SR_RXNE)) {
//                 if ((millis() - start_time) > timeout_ms) return false;
//             }
            
//             // Receive byte
//             uint8_t received = *(volatile uint8_t*)&SPI3->DR;
//             if (rx_data) {
//                 rx_data[i] = received;
//             }
//         }
        
//         // Wait for BSY flag to clear
//         while (SPI3->SR & SPI_SR_BSY) {
//             if ((millis() - start_time) > timeout_ms) return false;
//         }
        
//         return true;
//     }
    
//     /// Blocking USART transmit (non-DMA, polling mode)
//     /// @param data Transmit buffer
//     /// @param length Number of bytes to transmit
//     /// @param timeout_ms Timeout in milliseconds
//     /// @return true if transmission completed successfully
//     bool usart_transmit_blocking(const uint8_t* data, uint16_t length, uint32_t timeout_ms = 1000) {
//         USART_TypeDef* usart = get_usart();
//         if (!usart) {
//             return false;
//         }
        
//         uint32_t start_time = millis();
        
//         for (uint16_t i = 0; i < length; i++) {
//             // Wait for TXE flag
//             while (!(usart->ISR & USART_ISR_TXE)) {
//                 if ((millis() - start_time) > timeout_ms) return false;
//             }
            
//             // Send byte
//             usart->TDR = data[i];
//         }
        
//         // Wait for transmission complete
//         while (!(usart->ISR & USART_ISR_TC)) {
//             if ((millis() - start_time) > timeout_ms) return false;
//         }
        
//         return true;
//     }
    
//     /// Blocking USART receive (non-DMA, polling mode)
//     /// @param data Receive buffer
//     /// @param length Number of bytes to receive
//     /// @param timeout_ms Timeout in milliseconds
//     /// @return true if reception completed successfully
//     bool usart_receive_blocking(uint8_t* data, uint16_t length, uint32_t timeout_ms = 1000) {
//         USART_TypeDef* usart = get_usart();
//         if (!usart) {
//             return false;
//         }
        
//         uint32_t start_time = millis();
        
//         for (uint16_t i = 0; i < length; i++) {
//             // Wait for RXNE flag
//             while (!(usart->ISR & USART_ISR_RXNE)) {
//                 if ((millis() - start_time) > timeout_ms) return false;
//             }
            
//             // Receive byte
//             data[i] = (uint8_t)usart->RDR;
//         }
        
//         return true;
//     }
    
//     /// Check if USART has data available to read
//     /// @return true if data is available
//     bool usart_available() {
//         USART_TypeDef* usart = get_usart();
//         if (!usart) {
//             return false;
//         }
//         return (usart->ISR & USART_ISR_RXNE) != 0;
//     }
    
//     /// Read single byte from USART (non-blocking)
//     /// @param data Pointer to store received byte
//     /// @return true if byte was read successfully
//     bool usart_read_byte(uint8_t* data) {
//         USART_TypeDef* usart = get_usart();
//         if (!usart || !data) {
//             return false;
//         }
        
//         if (usart->ISR & USART_ISR_RXNE) {
//             *data = (uint8_t)usart->RDR;
//             return true;
//         }
        
//         return false;
//     }
    
//     /// Write single byte to USART (blocking)
//     /// @param data Byte to transmit
//     /// @param timeout_ms Timeout in milliseconds
//     /// @return true if byte was sent successfully
//     bool usart_write_byte(uint8_t data, uint32_t timeout_ms = 100) {
//         USART_TypeDef* usart = get_usart();
//         if (!usart) {
//             return false;
//         }
        
//         uint32_t start_time = millis();
        
//         // Wait for TXE flag
//         while (!(usart->ISR & USART_ISR_TXE)) {
//             if ((millis() - start_time) > timeout_ms) return false;
//         }
        
//         usart->TDR = data;
//         return true;
//     }
    
//     // ==============================================================================
//     // PERIPHERAL ACCESS HELPERS
//     // ==============================================================================
    
//     /// Get SPI3 peripheral pointer (only in SPI_MASTER mode)
//     /// @return Pointer to SPI3 or nullptr if not in SPI mode
//     SPI_TypeDef* get_spi() {
//         return (current_mode_ == Mode::SPI_MASTER) ? SPI3 : nullptr;
//     }
    
//     /// Get USART2 peripheral pointer (only in USART modes)
//     /// @return Pointer to USART2 or nullptr if not in USART mode
//     USART_TypeDef* get_usart() {
//         if (current_mode_ == Mode::USART_FULL || 
//             current_mode_ == Mode::USART_BASIC ||
//             current_mode_ == Mode::I2C_USART ||
//             current_mode_ == Mode::STEP_DIR_EN_USART) {
//             return USART2;
//         }
//         return nullptr;
//     }
    
//     /// Get I2C1 peripheral pointer (only in I2C modes)
//     /// @return Pointer to I2C1 or nullptr if not in I2C mode
//     I2C_TypeDef* get_i2c() {
//         if (current_mode_ == Mode::I2C_MASTER || current_mode_ == Mode::I2C_USART) {
//             return I2C1;
//         }
//         return nullptr;
//     }
    
//     /// Get TIM3 peripheral pointer (only in encoder modes)
//     /// @return Pointer to TIM3 or nullptr if not in encoder mode
//     TIM_TypeDef* get_timer() {
//         if (current_mode_ == Mode::ENCODER_ABZ || current_mode_ == Mode::ENCODER_PULSE_DIR) {
//             return TIM3;
//         }
//         return nullptr;
//     }
    
//     /// I2C write operation (blocking mode)
//     /// @param device_addr I2C device address (7-bit, shifted left by 1)
//     /// @param reg_addr Register address to write to
//     /// @param data Pointer to data buffer
//     /// @param length Number of bytes to write
//     /// @param timeout_ms Timeout in milliseconds
//     /// @return true if write completed successfully
//     bool i2c_write(uint8_t device_addr, uint8_t reg_addr, const uint8_t* data, uint16_t length, uint32_t timeout_ms = 100) {
//         I2C_TypeDef* i2c = get_i2c();
//         if (!i2c || !data) {
//             return false;
//         }
        
//         uint32_t start_time = millis();
        
//         // Wait until bus is ready
//         while (i2c->ISR & I2C_ISR_BUSY) {
//             if ((millis() - start_time) > timeout_ms) return false;
//         }
        
//         // Configure transfer: device address, write, 1 + length bytes (reg + data)
//         i2c->CR2 = 0;
//         i2c->CR2 = ((device_addr << 1) & I2C_CR2_SADD) |
//                    ((length + 1) << I2C_CR2_NBYTES_Pos) |
//                    I2C_CR2_AUTOEND | I2C_CR2_START;
        
//         // Send register address
//         while (!(i2c->ISR & I2C_ISR_TXIS)) {
//             if ((millis() - start_time) > timeout_ms) return false;
//         }
//         i2c->TXDR = reg_addr;
        
//         // Send data bytes
//         for (uint16_t i = 0; i < length; i++) {
//             while (!(i2c->ISR & I2C_ISR_TXIS)) {
//                 if ((millis() - start_time) > timeout_ms) return false;
//             }
//             i2c->TXDR = data[i];
//         }
        
//         // Wait for transfer complete
//         while (!(i2c->ISR & I2C_ISR_STOPF)) {
//             if ((millis() - start_time) > timeout_ms) return false;
//         }
        
//         // Clear STOP flag
//         i2c->ICR = I2C_ICR_STOPCF;
        
//         return true;
//     }
    
//     /// I2C read operation (blocking mode)
//     /// @param device_addr I2C device address (7-bit, shifted left by 1)
//     /// @param reg_addr Register address to read from
//     /// @param data Pointer to receive buffer
//     /// @param length Number of bytes to read
//     /// @param timeout_ms Timeout in milliseconds
//     /// @return true if read completed successfully
//     bool i2c_read(uint8_t device_addr, uint8_t reg_addr, uint8_t* data, uint16_t length, uint32_t timeout_ms = 100) {
//         I2C_TypeDef* i2c = get_i2c();
//         if (!i2c || !data) {
//             return false;
//         }
        
//         uint32_t start_time = millis();
        
//         // Wait until bus is ready
//         while (i2c->ISR & I2C_ISR_BUSY) {
//             if ((millis() - start_time) > timeout_ms) return false;
//         }
        
//         // Step 1: Write register address
//         i2c->CR2 = 0;
//         i2c->CR2 = ((device_addr << 1) & I2C_CR2_SADD) |
//                    (1 << I2C_CR2_NBYTES_Pos) |
//                    I2C_CR2_START;
        
//         // Send register address
//         while (!(i2c->ISR & I2C_ISR_TXIS)) {
//             if ((millis() - start_time) > timeout_ms) return false;
//         }
//         i2c->TXDR = reg_addr;
        
//         // Wait for transfer complete
//         while (!(i2c->ISR & I2C_ISR_TC)) {
//             if ((millis() - start_time) > timeout_ms) return false;
//         }
        
//         // Step 2: Read data
//         i2c->CR2 = 0;
//         i2c->CR2 = ((device_addr << 1) & I2C_CR2_SADD) |
//                    (length << I2C_CR2_NBYTES_Pos) |
//                    I2C_CR2_AUTOEND | I2C_CR2_START | I2C_CR2_RD_WRN;
        
//         // Receive data bytes
//         for (uint16_t i = 0; i < length; i++) {
//             while (!(i2c->ISR & I2C_ISR_RXNE)) {
//                 if ((millis() - start_time) > timeout_ms) return false;
//             }
//             data[i] = i2c->RXDR;
//         }
        
//         // Wait for stop
//         while (!(i2c->ISR & I2C_ISR_STOPF)) {
//             if ((millis() - start_time) > timeout_ms) return false;
//         }
        
//         // Clear STOP flag
//         i2c->ICR = I2C_ICR_STOPCF;
        
//         return true;
//     }
    
//     /// Check I2C device presence
//     /// @param device_addr I2C device address (7-bit, shifted left by 1)
//     /// @param timeout_ms Timeout in milliseconds
//     /// @return true if device responded with ACK
//     bool i2c_device_present(uint8_t device_addr, uint32_t timeout_ms = 10) {
//         I2C_TypeDef* i2c = get_i2c();
//         if (!i2c) {
//             return false;
//         }
        
//         uint32_t start_time = millis();
        
//         // Wait until bus is ready
//         while (i2c->ISR & I2C_ISR_BUSY) {
//             if ((millis() - start_time) > timeout_ms) return false;
//         }
        
//         // Send address with 0 bytes
//         i2c->CR2 = 0;
//         i2c->CR2 = ((device_addr << 1) & I2C_CR2_SADD) |
//                    I2C_CR2_AUTOEND | I2C_CR2_START;
        
//         // Wait for NACK or STOP
//         while (!(i2c->ISR & (I2C_ISR_STOPF | I2C_ISR_NACKF))) {
//             if ((millis() - start_time) > timeout_ms) return false;
//         }
        
//         bool present = !(i2c->ISR & I2C_ISR_NACKF);
        
//         // Clear flags
//         i2c->ICR = I2C_ICR_STOPCF | I2C_ICR_NACKCF;
        
//         return present;
//     }
    
//     /// Get detailed status information
//     /// @return Status string describing current mode and configuration
//     const char* get_status_string() const {
//         switch (current_mode_) {
//             case Mode::DISABLED: return "DISABLED";
//             case Mode::SPI_MASTER: return "SPI_MASTER";
//             case Mode::USART_FULL: return "USART_FULL";
//             case Mode::USART_BASIC: return "USART_BASIC";
//             case Mode::I2C_MASTER: return "I2C_MASTER";
//             case Mode::I2C_USART: return "I2C_USART";
//             case Mode::STEP_DIR_EN_ERR: return "STEP_DIR_EN_ERR";
//             case Mode::STEP_DIR_EN_PIN: return "STEP_DIR_EN_PIN";
//             case Mode::STEP_DIR_EN_USART: return "STEP_DIR_EN_USART";
//             case Mode::ENCODER_ABZ: return "ENCODER_ABZ";
//             case Mode::ENCODER_PULSE_DIR: return "ENCODER_PULSE_DIR";
//             case Mode::GPIO_INDIVIDUAL: return "GPIO_INDIVIDUAL";
//             default: return "UNKNOWN";
//         }
//     }
    
//     /// Print detailed configuration info (requires Serial/SerialUSB)
//     void print_config() const {
//         TP_LOG("=== Control Connector Configuration ===");
//         TP_LOG_VAL("Mode: ", get_status_string());
//         TP_LOG_VAL("Power enabled: ", power_enabled_);
        
//         if (current_mode_ == Mode::SPI_MASTER) {
//             TP_LOG("SPI Configuration:");
//             TP_LOG_VAL("  Prescaler: ", spi_config_.baudrate_prescaler);
//             TP_LOG_VAL("  CPOL: ", spi_config_.cpol);
//             TP_LOG_VAL("  CPHA: ", spi_config_.cpha);
//             TP_LOG_VAL("  Data size: ", spi_config_.datasize);
//             TP_LOG_VAL("  DMA enabled: ", spi_config_.use_dma);
//         }
        
//         if (current_mode_ == Mode::USART_FULL || current_mode_ == Mode::USART_BASIC ||
//             current_mode_ == Mode::I2C_USART || current_mode_ == Mode::STEP_DIR_EN_USART) {
//             TP_LOG("USART Configuration:");
//             TP_LOG_VAL("  Baudrate: ", usart_config_.baudrate);
//             TP_LOG_VAL("  Parity: ", usart_config_.parity_enable);
//             TP_LOG_VAL("  DMA TX: ", usart_config_.use_dma_tx);
//             TP_LOG_VAL("  DMA RX: ", usart_config_.use_dma_rx);
//         }
        
//         if (current_mode_ == Mode::I2C_MASTER || current_mode_ == Mode::I2C_USART) {
//             TP_LOG("I2C Configuration:");
//             TP_LOG_VAL("  Speed (Hz): ", i2c_config_.speed_hz);
//             TP_LOG_VAL("  Timing: 0x", i2c_config_.timing);
//         }
        
//         if (current_mode_ == Mode::ENCODER_ABZ || current_mode_ == Mode::ENCODER_PULSE_DIR) {
//             TP_LOG("Encoder Configuration:");
//             TP_LOG_VAL("  PPR: ", encoder_config_.pulses_per_revolution);
//             TP_LOG_VAL("  Velocity enabled: ", encoder_config_.enable_velocity);
//             TP_LOG_VAL("  Sample period (us): ", encoder_config_.velocity_sample_us);
//         }
        
//         TP_LOG("=====================================");
//     }
    
//     /// Get encoder position (only in encoder modes)
//     /// @return Current encoder count value
//     uint32_t get_encoder_position() {
//         if (current_mode_ == Mode::ENCODER_ABZ || current_mode_ == Mode::ENCODER_PULSE_DIR) {
//             return TIM3->CNT;
//         }
//         return 0;
//     }
    
//     /// Reset encoder position to zero (only in encoder modes)
//     void reset_encoder_position() {
//         if (current_mode_ == Mode::ENCODER_ABZ || current_mode_ == Mode::ENCODER_PULSE_DIR) {
//             TIM3->CNT = 0;
//             last_encoder_count_ = 0;
//         }
//     }
    
//     /// Update encoder velocity calculation (call periodically from main loop or timer)
//     /// Uses time-based velocity measurement between position samples
//     void update_encoder_velocity() {
//         if (current_mode_ != Mode::ENCODER_ABZ && current_mode_ != Mode::ENCODER_PULSE_DIR) {
//             return;
//         }
        
//         if (!encoder_config_.enable_velocity) {
//             return;
//         }
        
//         uint32_t current_time_us = micros();  // Get current time in microseconds
//         uint32_t current_count = TIM3->CNT;   // Read current encoder position
        
//         // Calculate time delta
//         uint32_t time_delta_us = current_time_us - last_encoder_time_us_;
        
//         // Only update if enough time has passed
//         if (time_delta_us >= encoder_config_.velocity_sample_us) {
//             // Calculate position delta (handle 16-bit overflow)
//             int32_t count_delta = (int32_t)((int16_t)(current_count - last_encoder_count_));
            
//             // Apply direction inversion if configured
//             if (encoder_config_.invert_direction) {
//                 count_delta = -count_delta;
//             }
            
//             // Calculate velocity in counts per second
//             // velocity = (count_delta / time_delta_us) * 1000000
//             if (time_delta_us > 0) {
//                 encoder_velocity_cps_ = (count_delta * 1000000) / (int32_t)time_delta_us;
//             }
            
//             // Update last values
//             last_encoder_count_ = current_count;
//             last_encoder_time_us_ = current_time_us;
//         }
//     }
    
//     /// Get encoder velocity in counts per second
//     /// @return Velocity in counts/second (can be negative for reverse direction)
//     int32_t get_encoder_velocity_cps() const {
//         return encoder_velocity_cps_;
//     }
    
//     /// Get encoder velocity in RPM
//     /// @return Velocity in revolutions per minute
//     float get_encoder_velocity_rpm() const {
//         if (encoder_config_.pulses_per_revolution == 0) {
//             return 0.0f;
//         }
//         // Convert counts/second to RPM: (cps / ppr) * 60
//         return (float)encoder_velocity_cps_ * 60.0f / (float)encoder_config_.pulses_per_revolution;
//     }
    
//     /// Get time period between encoder pulses in microseconds
//     /// Useful for high-resolution velocity measurement at low speeds
//     /// @return Period in microseconds (0 if no recent pulses)
//     uint32_t get_encoder_pulse_period_us() const {
//         return pulse_period_us_;
//     }
    
//     /// Update pulse period measurement (call from encoder input capture ISR if available)
//     /// This provides alternative velocity measurement using pulse timing
//     void update_pulse_period() {
//         uint32_t current_time = micros();
        
//         if (last_pulse_time_us_ != 0) {
//             pulse_period_us_ = current_time - last_pulse_time_us_;
//         }
        
//         last_pulse_time_us_ = current_time;
//     }
    
//     /// SPI chip select control (only in SPI_MASTER mode)
//     /// @param select true to select (CS low), false to deselect (CS high)
//     void spi_chip_select(bool select) {
//         if (current_mode_ == Mode::SPI_MASTER) {
//             if (select) {
//                 GPIOA->BRR = GPIO_PIN_15;   // CS low (active)
//             } else {
//                 GPIOA->BSRR = GPIO_PIN_15;  // CS high (inactive)
//             }
//         }
//     }
    
//     /// SPI DMA transfer (only in SPI_MASTER mode with DMA enabled)
//     /// @param tx_data Pointer to transmit buffer (can be nullptr for RX-only)
//     /// @param rx_data Pointer to receive buffer (can be nullptr for TX-only)
//     /// @param length Number of bytes to transfer
//     /// @return true if transfer started successfully
//     bool spi_transfer_dma(const uint8_t* tx_data, uint8_t* rx_data, uint16_t length) {
//         if (current_mode_ != Mode::SPI_MASTER || !spi_config_.use_dma) {
//             return false;
//         }
        
//         if (length > MAX_DMA_BUFFER) {
//             length = MAX_DMA_BUFFER;
//         }
        
//         // Copy TX data to buffer if provided
//         if (tx_data != nullptr) {
//             for (uint16_t i = 0; i < length; i++) {
//                 spi_tx_buffer_[i] = tx_data[i];
//             }
//         } else {
//             // Send dummy bytes if no TX data
//             for (uint16_t i = 0; i < length; i++) {
//                 spi_tx_buffer_[i] = 0xFF;
//             }
//         }
        
//         // Start DMA transfer
//         return start_spi_dma_transfer(length);
//     }
    
//     /// Check if SPI DMA transfer is complete
//     /// @return true if transfer is complete
//     bool spi_dma_is_complete() {
//         if (current_mode_ != Mode::SPI_MASTER || !spi_config_.use_dma) {
//             return true;
//         }
//         return (DMA1->ISR & DMA_ISR_TCIF4) != 0;  // Check DMA1 Channel 4 (SPI3_RX) complete flag
//     }
    
//     /// Get received data from SPI DMA buffer
//     /// @param rx_data Pointer to destination buffer
//     /// @param length Number of bytes to copy
//     void spi_get_rx_data(uint8_t* rx_data, uint16_t length) {
//         if (rx_data != nullptr && length <= MAX_DMA_BUFFER) {
//             for (uint16_t i = 0; i < length; i++) {
//                 rx_data[i] = spi_rx_buffer_[i];
//             }
//         }
//     }
    
//     /// USART DMA transmit (only in USART modes with DMA TX enabled)
//     /// @param data Pointer to transmit buffer
//     /// @param length Number of bytes to transmit
//     /// @return true if transmission started successfully
//     bool usart_transmit_dma(const uint8_t* data, uint16_t length) {
//         if (!usart_config_.use_dma_tx) {
//             return false;
//         }
        
//         if (length > MAX_DMA_BUFFER) {
//             length = MAX_DMA_BUFFER;
//         }
        
//         // Copy data to TX buffer
//         for (uint16_t i = 0; i < length; i++) {
//             usart_tx_buffer_[i] = data[i];
//         }
        
//         // Start DMA transmission
//         return start_usart_tx_dma(length);
//     }
    
//     /// Check if USART DMA transmission is complete
//     /// @return true if transmission is complete
//     bool usart_tx_dma_is_complete() {
//         if (!usart_config_.use_dma_tx) {
//             return true;
//         }
//         return (DMA1->ISR & DMA_ISR_TCIF7) != 0;  // Check DMA1 Channel 7 (USART2_TX) complete flag
//     }
    
//     /// USART DMA receive (only in USART modes with DMA RX enabled)
//     /// @param length Number of bytes to receive
//     /// @return true if reception started successfully
//     bool usart_receive_dma(uint16_t length) {
//         if (!usart_config_.use_dma_rx) {
//             return false;
//         }
        
//         if (length > MAX_DMA_BUFFER) {
//             length = MAX_DMA_BUFFER;
//         }
        
//         // Start DMA reception
//         return start_usart_rx_dma(length);
//     }
    
//     /// Check if USART DMA reception is complete
//     /// @return true if reception is complete
//     bool usart_rx_dma_is_complete() {
//         if (!usart_config_.use_dma_rx) {
//             return true;
//         }
//         return (DMA1->ISR & DMA_ISR_TCIF6) != 0;  // Check DMA1 Channel 6 (USART2_RX) complete flag
//     }
    
//     /// Get received data from USART DMA buffer
//     /// @param rx_data Pointer to destination buffer
//     /// @param length Number of bytes to copy
//     void usart_get_rx_data(uint8_t* rx_data, uint16_t length) {
//         if (rx_data != nullptr && length <= MAX_DMA_BUFFER) {
//             for (uint16_t i = 0; i < length; i++) {
//                 rx_data[i] = usart_rx_buffer_[i];
//             }
//         }
//     }
    
//     /// Set enable pin state (for step-dir modes)
//     /// @param enable true to enable motor, false to disable
//     void set_enable(bool enable) {
//         if (current_mode_ == Mode::STEP_DIR_EN_ERR ||
//             current_mode_ == Mode::STEP_DIR_EN_PIN ||
//             current_mode_ == Mode::STEP_DIR_EN_USART) {
//             write_pin(15, enable); // PA_15 is the enable pin
//         }
//     }
    
//     /// Set error pin state (for STEP_DIR_EN_ERR mode)
//     /// @param error true to signal error, false for normal operation
//     void set_error(bool error) {
//         if (current_mode_ == Mode::STEP_DIR_EN_ERR) {
//             write_pin(3, error); // PB_3 is the error pin
//         }
//     }
    
//     /// Read step pin state (for step-dir modes)
//     /// @return true if step pin is high
//     bool read_step() {
//         if (current_mode_ == Mode::STEP_DIR_EN_ERR ||
//             current_mode_ == Mode::STEP_DIR_EN_PIN) {
//             return read_pin(4); // PB_4 is step pin
//         }
//         return false;
//     }
    
//     /// Read direction pin state (for step-dir modes)
//     /// @return true if direction is positive (CW)
//     bool read_direction() {
//         if (current_mode_ == Mode::STEP_DIR_EN_ERR ||
//             current_mode_ == Mode::STEP_DIR_EN_PIN ||
//             current_mode_ == Mode::STEP_DIR_EN_USART) {
//             return read_pin(5); // PB_5 is direction pin
//         }
//         return false;
//     }
// };

// } // namespace ControlConnector

// #endif // CNTRL_SETUP_H

// /**
//  * ============================================================================
//  * USAGE EXAMPLES - Control Connector Setup
//  * ============================================================================
//  * 
//  * EXAMPLE 1: Quick SPI setup (5MHz, Mode 0, with DMA)
//  * ----------------------------------------------------------------------------
//  * ControlConnector::ControlSetup ctrl;
//  * ctrl.init();
//  * ctrl.set_power(true);
//  * 
//  * if (ctrl.setup_spi_quick(5.0f, 0, true)) {  // 5MHz, Mode 0, DMA enabled
//  *     uint8_t tx_data[4] = {0xA0, 0x00, 0x00, 0x00};
//  *     uint8_t rx_data[4];
//  *     
//  *     ctrl.spi_chip_select(true);
//  *     ctrl.spi_transfer_dma(tx_data, nullptr, 4);
//  *     while (!ctrl.spi_dma_is_complete()) { }
//  *     ctrl.spi_get_rx_data(rx_data, 4);
//  *     ctrl.spi_chip_select(false);
//  *     
//  *     TP_LOG_ARRAY("RX Data: ", rx_data);
//  * }
//  * 
//  * EXAMPLE 2: Quick USART setup with DMA
//  * ----------------------------------------------------------------------------
//  * if (ctrl.setup_usart_quick(921600, true)) {  // 921600 baud, DMA enabled
//  *     const uint8_t msg[] = "Hello from USART\r\n";
//  *     ctrl.usart_transmit_dma(msg, sizeof(msg) - 1);
//  *     while (!ctrl.usart_tx_dma_is_complete()) { }
//  * }
//  * 
//  * EXAMPLE 3: USART with blocking operations (no DMA)
//  * ----------------------------------------------------------------------------
//  * if (ctrl.setup_usart_quick(115200, false)) {  // 115200 baud, no DMA
//  *     const uint8_t msg[] = "Test\r\n";
//  *     ctrl.usart_transmit_blocking(msg, sizeof(msg) - 1);
//  *     
//  *     // Echo received data
//  *     while (true) {
//  *         if (ctrl.usart_available()) {
//  *             uint8_t byte;
//  *             if (ctrl.usart_read_byte(&byte)) {
//  *                 ctrl.usart_write_byte(byte);  // Echo back
//  *             }
//  *         }
//  *     }
//  * }
//  * 
//  * EXAMPLE 4: I2C sensor communication
//  * ----------------------------------------------------------------------------
//  * if (ctrl.setup_i2c_quick(400)) {  // 400kHz Fast Mode
//  *     uint8_t device_addr = 0x68;  // Example: MPU6050
//  *     
//  *     // Check if device is present
//  *     if (ctrl.i2c_device_present(device_addr)) {
//  *         TP_LOG("Device found!");
//  *         
//  *         // Write to register
//  *         uint8_t reg_addr = 0x6B;  // PWR_MGMT_1
//  *         uint8_t data = 0x00;      // Wake up
//  *         ctrl.i2c_write(device_addr, reg_addr, &data, 1);
//  *         
//  *         // Read from register
//  *         uint8_t who_am_i;
//  *         ctrl.i2c_read(device_addr, 0x75, &who_am_i, 1);
//  *         TP_LOG_VAL("WHO_AM_I: 0x", who_am_i);
//  *     }
//  * }
//  * 
//  * EXAMPLE 5: Encoder with velocity measurement
//  * ----------------------------------------------------------------------------
//  * if (ctrl.setup_encoder_quick(2048, true, 100)) {  // 2048 PPR, 100Hz sampling
//  *     // In main loop:
//  *     while (true) {
//  *         ctrl.update_encoder_velocity();  // Call at consistent rate
//  *         
//  *         uint32_t position = ctrl.get_encoder_position();
//  *         int32_t velocity_cps = ctrl.get_encoder_velocity_cps();
//  *         float velocity_rpm = ctrl.get_encoder_velocity_rpm();
//  *         
//  *         TP_LOG_VALS("Pos:", position, "Vel(CPS):", velocity_cps, "RPM:", velocity_rpm);
//  *         
//  *         delay(10);  // 100Hz loop
//  *     }
//  * }
//  * 
//  * EXAMPLE 6: Runtime mode switching
//  * ----------------------------------------------------------------------------
//  * // Start with SPI
//  * ctrl.setup_spi_quick(10.0f);
//  * // ... do SPI operations
//  * 
//  * // Switch to USART at runtime
//  * ctrl.setup_usart_quick(115200);
//  * // ... do USART operations
//  * 
//  * // Switch to I2C
//  * ctrl.setup_i2c_quick(100);
//  * // ... do I2C operations
//  * 
//  * // Each switch automatically deinitializes previous mode safely
//  * 
//  * EXAMPLE 7: Runtime parameter changes
//  * ----------------------------------------------------------------------------
//  * ctrl.setup_usart_quick(115200);
//  * 
//  * // Change baud rate on the fly
//  * ControlConnector::USARTConfig cfg = ctrl.get_usart_config();
//  * cfg.baudrate = 921600;
//  * ctrl.configure_usart(cfg);  // Automatically reconfigures USART
//  * 
//  * // Continue using with new baud rate
//  * 
//  * EXAMPLE 8: Advanced SPI with custom configuration
//  * ----------------------------------------------------------------------------
//  * ControlConnector::SPIConfig spi_cfg;
//  * spi_cfg.baudrate_prescaler = 2;  // /8 divider (~21MHz)
//  * spi_cfg.cpol = true;             // Clock idle high
//  * spi_cfg.cpha = true;             // Second edge capture
//  * spi_cfg.datasize = 8;
//  * spi_cfg.use_dma = true;
//  * spi_cfg.dma_buffer_size = 128;
//  * 
//  * ctrl.configure_spi(spi_cfg);
//  * ctrl.switch_mode(ControlConnector::Mode::SPI_MASTER);
//  * 
//  * // DMA transfer
//  * uint8_t tx[128], rx[128];
//  * for (int i = 0; i < 128; i++) tx[i] = i;
//  * 
//  * ctrl.spi_chip_select(true);
//  * ctrl.spi_transfer_dma(tx, nullptr, 128);
//  * while (!ctrl.spi_dma_is_complete()) { }
//  * ctrl.spi_get_rx_data(rx, 128);
//  * ctrl.spi_chip_select(false);
//  * 
//  * EXAMPLE 9: Mixed I2C + USART mode
//  * ----------------------------------------------------------------------------
//  * if (ctrl.switch_mode(ControlConnector::Mode::I2C_USART)) {
//  *     // Configure both peripherals
//  *     ControlConnector::I2CConfig i2c_cfg;
//  *     i2c_cfg.speed_hz = 400000;
//  *     ctrl.configure_i2c(i2c_cfg);
//  *     
//  *     ControlConnector::USARTConfig uart_cfg;
//  *     uart_cfg.baudrate = 115200;
//  *     ctrl.configure_usart(uart_cfg);
//  *     
//  *     // Use both simultaneously
//  *     uint8_t sensor_data;
//  *     ctrl.i2c_read(0x68, 0x3B, &sensor_data, 1);
//  *     
//  *     char msg[50];
//  *     sprintf(msg, "Sensor: %d\r\n", sensor_data);
//  *     ctrl.usart_transmit_blocking((uint8_t*)msg, strlen(msg));
//  * }
//  * 
//  * EXAMPLE 10: Debug and status
//  * ----------------------------------------------------------------------------
//  * ctrl.setup_spi_quick(5.0f, 0, true);
//  * 
//  * // Print current configuration
//  * ctrl.print_config();
//  * 
//  * // Get mode name
//  * TP_LOG_VAL("Current mode: ", ctrl.get_status_string());
//  * TP_LOG_VAL("Power enabled: ", ctrl.is_power_enabled());
//  * 
//  * // Check peripheral pointers
//  * if (ctrl.get_spi() != nullptr) {
//  *     TP_LOG("SPI peripheral is active");
//  * }
//  */
