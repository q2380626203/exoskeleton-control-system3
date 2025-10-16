# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ESP32-based dual motor control system using Unitree GO-M8010-6 motors with RS485 communication. The system implements a sophisticated speed-following state machine for coordinated bipedal walking motion control.

## Build & Flash Commands

```bash
# Build the project
idf.py build

# Flash to ESP32
idf.py flash

# Monitor serial output
idf.py monitor

# Build, flash and monitor in one command
idf.py -p PORT flash monitor
```

## Architecture

### Core Components

**Motor Control System** (`unitree_motor.cpp/h`)
- Implements GO-M8010-6 protocol (RS485, 4Mbps, CRC16-CCITT)
- Synchronous send/receive at 500Hz control loop
- Frame format: 16-byte command, 16-byte response
- Motor IDs: 0x01 (Motor 1), 0x02 (Motor 2)
- UART2: TX=GPIO13, RX=GPIO12

**Speed Follow State Machine** (`speed_follow_mode.cpp/h`)
- 6-state machine: IDLE → WAITING → WORKING → PHASE1 → PHASE2 → IDLE (cycle)
- **PHASE1** (Lift): Dynamic velocity following using `motor_vel * 0.8` in real-time
- **PHASE2** (Press): Dynamic velocity following, completes when velocity returns to ±0.5 rad/s threshold
- Key parameters (adjustable in `init()`):
  - `_phase1_timeout_ms`: PHASE1 timeout (default 500ms)
  - `_phase2_timeout_ms`: PHASE2 timeout (default 350ms)
  - `_velocity_scale`: Velocity scaling factor (default 0.8)
  - `_phase2_vel_threshold`: PHASE2 completion threshold (default ±0.5 rad/s)
- **Critical**: `update()` is called twice per loop (motor_data_1, motor_data_2), must check `motor_data.id` to update correct motor

**Position Buffer & Wave Analysis** (`position_buffer.c/h`)
- Circular buffers for motor positions (400 samples)
- Peak/valley detection for gait analysis
- Sliding window filtering
- Outputs ch6/ch7 differential values for speed-follow triggering

**WiFi Web Server** (`wifi_webserver.c/h`, `webpage.h`)
- AP Mode: SSID "ESP32_Motor_Control", IP 192.168.4.1
- Real-time parameter adjustment via HTTP endpoints
- Motor config exposure through `getMotorConfig()`

### Control Flow

```
app_main() → WiFi init → Web server → Motor driver init → motor_control_task()
    │
    └─→ motor_control_task() @ 500Hz:
         1. Read global_motor_1/2 params (mutex protected)
         2. sendRecv() to both motors (sequential, ~2ms total)
         3. Position buffer update
         4. Wave analysis (ch6/ch7 calculation)
         5. speed_follow.update(motor_data_1) ← First call
         6. speed_follow.update(motor_data_2) ← Second call
         7. checkThresholdAndActivate()
         8. printf() motor data
```

### State Machine Trigger Logic

1. **Activation**: ch6_max or ch7_max > threshold → WAITING state
2. **WAITING** (300ms): Detect motor velocity trigger → PHASE1
3. **PHASE1** (Lift leg):
   - Continuously updates motor params using real-time velocity * 0.8
   - Early exit on velocity reversal detection
   - Timeout: 500ms → force PHASE2
4. **PHASE2** (Press leg):
   - Continuously updates motor params using real-time velocity * 0.8
   - Early exit when velocity drops to [-0.5, +0.5] range
   - Timeout: 350ms → force IDLE
5. **IDLE** (200ms) → WORKING → detect next trigger → cycle repeats

### Data Flow (per 2ms cycle)

```
UART RX → unpack_motor_data_go_m8010_6() → MotorDataA1{id, pos, vel, t, temp, ...}
    ↓
position_buffer_add_motorX() → wave analysis → ch6/ch7_max
    ↓
speed_follow.update(motor_data_X) → checks motor_data.id → updates corresponding motor
    ↓
setMotorParams() → modifies global_motor_1/2 (mutex) → next cycle reads new params
```

## Important Implementation Details

### Motor Parameter Updates in PHASE1/PHASE2

The speed-following logic dynamically adjusts motor velocity parameters based on real-time feedback:

```cpp
// In PHASE1/PHASE2, velocity is continuously updated:
if (_lifting_motor == 1 && motor_data.id == 1) {
    float scaled_vel = motor_data.vel * _velocity_scale;  // Real-time velocity * 0.8
    setMotorParams(1, phase.mode, phase.pos, scaled_vel, phase.torque, phase.kp, phase.kd);
}
```

This means the motor's commanded velocity adapts every cycle based on its actual measured velocity, creating a feedback loop.

### Mutex Protection

All access to `global_motor_1` and `global_motor_2` must use `motor_params_mutex`:
- Read in `motor_control_task()` before sending commands
- Write in `speed_follow.setMotorParams()` and web handlers

### CRC Validation

Currently CRC check failures are logged but not enforced (see `unpack_motor_data_go_m8010_6:155`). If enabling strict CRC validation, uncomment the `return false;` line.

## Configuration Parameters

Key adjustable parameters in `speed_follow_mode.cpp::init()`:
- `trigger_speed`: ±2.0 rad/s (velocity threshold for triggering PHASE1)
- `waiting_duration_ms`: 300ms (delay after ch6/ch7 threshold trigger)
- `idle_duration_ms`: 200ms (rest period between cycles)
- Dynamic params: `_velocity_scale`, `_phase1_timeout_ms`, `_phase2_timeout_ms`, `_phase2_vel_threshold`

Motor PHASE parameters (vel, torque, kp, kd) can be adjusted via web interface or by modifying `init()` defaults.

## Common Pitfalls

1. **Forgetting motor ID check**: When adding logic in `update()`, always check `motor_data.id` to avoid applying Motor 1's data to Motor 2
2. **Blocking operations in control loop**: The 500Hz loop is tight (2ms period), avoid delays or heavy computation
3. **Missing mutex locks**: Any read/write to `global_motor_X` outside the control loop must lock `motor_params_mutex`
4. **UART buffer overflow**: At 4Mbps with 16-byte frames, timing is critical; delays cause data loss
5. **State machine race conditions**: Since `update()` is called twice per cycle, state transitions can occur mid-cycle; use `motor_data.id` checks carefully

## See Also

- 状态机逻辑.md: Detailed state machine flow diagram (in Chinese)
- sdkconfig: ESP-IDF configuration (UART, WiFi, task stack sizes)
