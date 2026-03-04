# Motor Control Layer Refactoring - Configuration Guide

## Overview
The motor control layer has been refactored to use direct hardware control via H-Bridge drivers (PWM) and quadrature encoders connected to the ESP32-S3, replacing the previous UART-based smart driver implementation.

## New Files Created

### 1. `inc/encoder.h` & `main/motion_control/encoder.c`
- **Purpose**: Quadrature encoder reading using ESP-IDF PCNT (Pulse Counter) peripheral
- **Resolution**: 2464 pulses per revolution (56 gear ratio × 11 motor PPR × 4 quadrature edges)
- **Features**:
  - Reads encoder position (radians)
  - Calculates velocity (rad/s)
  - Supports 4 encoders (one per motor)

### 2. `inc/pid.h` & `main/motion_control/pid.c`
- **Purpose**: Software PID controller for closed-loop motor speed control
- **Features**:
  - Proportional, Integral, and Derivative control
  - Anti-windup protection
  - Configurable gains and output limits
  - Auto-calculated sample time or fixed interval

### 3. Updated `inc/motor_driver.h` & `main/motion_control/motor_driver.c`
- **Purpose**: Motor control using MCPWM peripheral for PWM generation
- **Features**:
  - 4 independent motor controllers
  - 20 kHz PWM frequency (configurable)
  - Direction control via GPIO
  - Automatic PID control loop at 100 Hz
  - Compatible with existing API (`SetWheelSpeed`, `GetWheelInfor`, etc.)

## GPIO Pin Configuration

**IMPORTANT**: You must configure the GPIO pins in the header files before building!

### Encoder Pins (`inc/encoder.h`)
```c
// Motor 1 Encoder
#define ENCODER_1_PHASE_A_GPIO XX  // Replace XX with your GPIO number
#define ENCODER_1_PHASE_B_GPIO XX  // Replace XX with your GPIO number

// Motor 2 Encoder
#define ENCODER_2_PHASE_A_GPIO XX
#define ENCODER_2_PHASE_B_GPIO XX

// Motor 3 Encoder
#define ENCODER_3_PHASE_A_GPIO XX
#define ENCODER_3_PHASE_B_GPIO XX

// Motor 4 Encoder
#define ENCODER_4_PHASE_A_GPIO XX
#define ENCODER_4_PHASE_B_GPIO XX
```

### Motor PWM and Direction Pins (`inc/motor_driver.h`)
```c
// Motor 1 (H-Bridge)
#define MOTOR_1_PWM_GPIO XX  // PWM signal to H-Bridge
#define MOTOR_1_DIR_GPIO XX  // Direction control (or use separate IN1/IN2)

// Motor 2 (H-Bridge)
#define MOTOR_2_PWM_GPIO XX
#define MOTOR_2_DIR_GPIO XX

// Motor 3 (H-Bridge)
#define MOTOR_3_PWM_GPIO XX
#define MOTOR_3_DIR_GPIO XX

// Motor 4 (H-Bridge)
#define MOTOR_4_PWM_GPIO XX
#define MOTOR_4_DIR_GPIO XX
```

### Recommended GPIO Selection for ESP32-S3
- **Avoid**: GPIO 0 (boot), GPIO 45/46 (strapping pins)
- **Preferred for PWM**: GPIO 1-21, 35-48
- **Preferred for Encoders**: GPIO 1-21, 35-48 (with interrupt capability)
- **Note**: Ensure pins don't conflict with WiFi, SPI, or other peripherals in use

## PID Tuning

Default PID gains are set in `inc/motor_driver.h`:
```c
#define DEFAULT_KP 1.0f   // Proportional gain
#define DEFAULT_KI 0.5f   // Integral gain
#define DEFAULT_KD 0.1f   // Derivative gain
```

### Tuning Procedure
1. Start with conservative values (low gains)
2. Increase Kp until the system responds quickly but doesn't oscillate
3. Add Ki to eliminate steady-state error
4. Add Kd to reduce overshoot and improve stability
5. Test at various speeds and loads

### Accessing PID Controllers
To modify PID gains at runtime (advanced):
```c
// In motor_driver.c, motors[] array contains PID controllers
// You can add a function to update gains if needed
```

## API Compatibility

The following functions maintain the same interface as before:

- `motor_driver_init()` - Initialize motors, encoders, and PID controllers
- `SetWheelSpeed(float *WheelSpeed)` - Set target speeds for all 4 wheels (rad/s)
- `SetSingleWheelSpeed(wheel_addr_t wheel_addr, int speed)` - Set single wheel speed
- `MecanumSpeedControl(float theta, float vx, float vy, float omega_theta)` - Mecanum control
- `GetWheelInfor(wheel_infor_t *wheel_infor)` - Get encoder feedback

### Removed Functions
- `SendCommand()` - No longer needed (was for UART communication)
- `StartPid()` - No longer needed (PID is always running)
- `ConfigureDriverSilentMode()` - Driver-specific, not applicable

## Control Loop Architecture

```
┌─────────────────────────────────────────────────────┐
│  motor_control_task (100 Hz, Priority 5)           │
│  ┌───────────────────────────────────────────────┐ │
│  │ 1. encoder_update()                           │ │
│  │    - Read PCNT counters                       │ │
│  │    - Calculate velocity & position            │ │
│  │                                                │ │
│  │ 2. For each motor (0-3):                      │ │
│  │    - Get current velocity from encoder        │ │
│  │    - PID_Compute(target, current) → PWM duty  │ │
│  │    - set_motor_pwm(duty_cycle)                │ │
│  └───────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────┘
```

## Hardware Connections

### H-Bridge Module (per motor)
- **ESP32 PWM GPIO** → H-Bridge PWM/Enable input
- **ESP32 DIR GPIO** → H-Bridge Direction input (or control IN1/IN2 separately)
- **H-Bridge OUT1/OUT2** → Motor terminals
- **Power Supply** → H-Bridge VCC (motor voltage, e.g., 12V/24V)
- **Ground** → Common ground with ESP32

### Quadrature Encoder (per motor)
- **Encoder Phase A** → ESP32 GPIO (defined in encoder.h)
- **Encoder Phase B** → ESP32 GPIO (defined in encoder.h)
- **Encoder VCC** → 3.3V or 5V (check encoder specs)
- **Encoder GND** → ESP32 GND

## Testing Procedure

1. **Configure GPIO pins** in `encoder.h` and `motor_driver.h`
2. **Build and flash** the firmware
3. **Verify encoder readings**:
   - Manually rotate each motor
   - Check console logs for position/velocity changes
4. **Test individual motors**:
   - Use `SetSingleWheelSpeed(WHEEL_1_ADDR, 10)` via socket command
   - Verify motor spins at controlled speed
5. **Tune PID gains**:
   - Adjust DEFAULT_KP, DEFAULT_KI, DEFAULT_KD
   - Observe step response and steady-state error
6. **Test full robot control**:
   - Send velocity commands via socket
   - Verify mecanum kinematics work correctly

## Troubleshooting

### Motors don't move
- Check H-Bridge wiring and power supply
- Verify PWM signal with oscilloscope
- Check GPIO pin configuration
- Ensure motor_control_task is running

### Encoders show no data
- Verify encoder wiring (Phase A, Phase B, power)
- Check PCNT configuration in logs
- Test encoder with multimeter/oscilloscope while rotating motor

### Unstable speed control
- Reduce PID gains (especially Kp and Kd)
- Increase control loop frequency if needed
- Check for mechanical issues (friction, binding)
- Verify encoder signal quality

### Compilation errors
- Ensure all new files are added to CMakeLists.txt
- Check for missing includes
- Verify ESP-IDF version compatibility (v5.x recommended)

## Performance Considerations

- **Control loop**: 100 Hz (10ms period) - adequate for most applications
- **Encoder update**: Called within control loop
- **PID computation**: ~10-20 µs per motor on ESP32-S3
- **Total overhead**: < 1ms per control cycle

## Future Enhancements

- [ ] Add velocity feedforward term to PID
- [ ] Implement current limiting/monitoring
- [ ] Add encoder direction detection auto-calibration
- [ ] Create runtime PID tuning interface via socket
- [ ] Add acceleration ramping for smoother control
- [ ] Implement fault detection (encoder failure, motor stall)

## Migration Checklist

- [x] Create encoder.h/encoder.c
- [x] Create pid.h/pid.c
- [x] Update motor_driver.h
- [x] Rewrite motor_driver.c
- [x] Remove UART dependencies
- [x] Update socket.c (remove StartPid command)
- [x] Maintain API compatibility
- [ ] Configure GPIO pins for your hardware
- [ ] Test encoder readings
- [ ] Tune PID controllers
- [ ] Verify full system operation

## Notes
- The encoder resolution (2464 PPR) is based on your specification: 56×11×4
- PWM frequency (20 kHz) is above audible range for quiet operation
- Control loop at 100 Hz provides good responsiveness for most robotics applications
- All GPIO pins are placeholders (set to 0) and **must** be configured for your hardware
