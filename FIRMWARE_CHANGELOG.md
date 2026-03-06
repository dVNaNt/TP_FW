# TunePulse Firmware Changelog

## 2026-02-09 - Error Management Refactoring with Set/Reset Pattern

### Changes
- **Error Manager Refactoring**:
  - Added `reset_*` methods for all fatal and alert errors
  - Removed automatic timestamp tracking and `fatal_active_`/`alert_active_` flags
  - Simplified error state management - errors are now purely flag-based
  - Removed automatic `tick()` timeout clearing - errors must be explicitly reset
  - `has_fatal()` and `has_alert()` now check flags directly instead of active state
- **Platform-Independent LED Control**:
  - Added LED control callback system in `lib.h` (no TARGET dependencies)
  - `led_callbacks_[3]` array in TunePulseHeart for RGB LED management
  - `set_led_callback()`, `led_control()`, and `get_led_callbacks()` methods
  - LED callbacks registered in `tunepulse_init()` for each target
- **Protection Recovery Integration**:
  - LED indication for overvoltage (red) and overcurrent (green) errors
  - All LEDs turn off after 200ms recovery timeout
- **Code Cleanup**:
  - Fixed include order in `feedback.h` (error.h before current_sense.h)
  - Fixed Channel enum usage in `driver_pwm.h`
  - Removed unused timestamp and active flag members from ErrorManager

### Firmware Size
- RAM:   19.2% (6296 bytes / 32768 bytes) - **-8 bytes**
- Flash: 43.8% (57396 bytes / 131072 bytes) - **-124 bytes**

### Technical Details
- Error reset methods should be called when comparator outputs go low
- Errors persist until explicitly reset (no automatic timeout clearing)
- LED control fully platform-independent through callback mechanism
- Reduced memory footprint by removing timestamp tracking
