# real_firmware — ESP32 Arduino Firmware
## Arduino IDE Setup
| Item | Version |
|---|---|
| Arduino IDE | 2.x |
| ESP32 board core | **2.0.2** (bc newer versions break micro-ROS timers) |
| micro_ros_arduino | jazzy release — install as .ZIP from GitHub |
| MPU6050 library | by Electronic Cats — via Library Manager |

Install ESP32 core 2.0.2:
Boards Manager URL: `https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json`
Search "esp32" → install version **2.0.2** specifically

Install micro_ros_arduino:
Download jazzy .ZIP from: `https://github.com/micro-ROS/micro_ros_arduino/releases`
Arduino IDE → Sketch → Include Library → Add .ZIP Library

## Wiring
### BTS7960 — Left Motor
| BTS7960 Pin | ESP32 GPIO |
|---|---|
| RPWM | 25 |
| LPWM | 26 |
| R_EN | 27 |
| L_EN | 14 |
| VCC  | 5V |
| GND  | GND |

### BTS7960 — Right Motor
| BTS7960 Pin | ESP32 GPIO |
|---|---|
| RPWM | 32 |
| LPWM | 33 |
| R_EN | 15 |
| L_EN | 4  |
| VCC  | 5V |
| GND  | GND |

### Encoders
The ISR reads both A and B on every CHANGE edge (X4 quadrature) to determine direction.

Effective resolution: 600 PPR × 4 edges × 30 gear ratio = **72 000 counts/rev** per wheel
Tick distance: 2 x PI × 0.065 m / 72 000 ≈ **0.0057 mm per tick**

| Signal          | ESP32 GPIO |
|-----------------|------------|
| Left encoder A  | 18         |
| Left encoder B  | 16         |
| Right encoder A | 19         |
| Right encoder B | 17         |
| VCC             | 3.3V       |
| GND             | GND        |

### MPU-6050
| MPU Pin | ESP32 GPIO |
|---|---|
| SDA | 21 |
| SCL | 22 |
| VCC | 3.3V |
| GND | GND |

### ESP32 → Pi 5
USB cable from ESP32 Dev Module to Pi 5 USB port

## udev Rules (run on Pi 5)
Gives fixed device names regardless of plug order:
```bash
# Find device IDs
udevadm info -a -n /dev/ttyUSB0 | grep idVendor
udevadm info -a -n /dev/ttyUSB1 | grep idVendor
# Create rules file
sudo nano /etc/udev/rules.d/99-robot.rules
```
(replace idVendor/idProduct with actual values):
Apply rules:
```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
```

## ROS 2 Topics
### Published by ESP32
| Topic | Type | Rate | Description |
|---|---|---|---|
| `/odom` | `nav_msgs/Odometry` | 20 Hz | Wheel-odometry pose and velocity |
| `/imu/data` | `sensor_msgs/Imu` | 50 Hz | Raw accelerometer + gyroscope |
| `/joint_states` | `sensor_msgs/JointState` | 20 Hz | Wheel positions (rad) and velocities (rad/s) |
| `/pid_debug` | `std_msgs/Float32MultiArray` | 20 Hz | PID internals |

### Subscribed by ESP32
| Topic | Type | Description |
|---|---|---|
| `/cmd_vel_stamped` | `geometry_msgs/TwistStamped` | Drive commands |
| `/pid_gains` | `std_msgs/String` | Live PID gain updates |

### /pid_debug array layout
| Index | Value |
|---|---|
| 0 | Left wheel slew-limited target (m/s) |
| 1 | Left wheel actual speed (m/s) |
| 2 | Left wheel error (target − actual) |
| 3 | Left wheel PID output (PWM) |
| 4 | Right wheel slew-limited target (m/s) |
| 5 | Right wheel actual speed (m/s) |
| 6 | Right wheel error (target − actual) |
| 7 | Right wheel PID output (PWM) |

In PlotJuggler: open `/pid_debug`, drag `data[0]` and `data[1]` onto the same panel
to see left-wheel target vs actual. `data[4]` and `data[5]` for the right wheel.
`data[2]` and `data[6]` show error converging to zero when the PID is well-tuned.
Note: `data[0]` and `data[4]` show the **slew-limited** target — the ramp the PID
actually sees — not the raw command from `/cmd_vel_stamped`.

## Motor Protection
These protection layers run simultaneously every odom tick (20 Hz):

### 1 — Stall Detection
**Problem:** If the robot is blocked by an obstacle, the PID pushes PWM toward 255
and holds it there. Sustained stall current burns motor windings.

**How it works:** Every tick the firmware checks whether PWM is above
`STALL_PWM_THRESHOLD` (220) while wheel velocity stays below
`STALL_SPEED_THRESHOLD` (0.02 m/s). If both conditions hold for longer than
`STALL_TIME_MS` (800 ms), both motors are cut immediately and all PID state is
reset. The robot stays stopped until a new `/cmd_vel_stamped` arrives.

**Tuning `STALL_TIME_MS`:** 800 ms is a good balance for indoor navigation near
obstacles — short enough to protect the motors quickly, long enough not to
false-trip during turns or carpet bumps. Shorten for more aggressive protection;
lengthen if the robot stops unexpectedly on rough terrain.

### 2 — Slew Rate Limiter (soft start / soft stop)
**Problem:** Jumping from 0 to full speed in one tick creates a large instantaneous
PID error, causing a high-current inrush spike and gearbox shock.

**How it works:** Raw target velocities from `/cmd_vel_stamped` are stored in
`target_v_left/right`. Each tick the `slew()` function advances `slewed_left/right`
toward the raw target by at most `MAX_ACCEL_PER_TICK` (0.05 m/s per tick).
The PID receives `slewed_*` instead of the raw target so speed changes are
always gradual.

`MAX_ACCEL_PER_TICK = 0.05 m/s × 20 Hz = 1.0 m/s²` maximum acceleration.
Raise in `config.h` if the robot feels sluggish; lower if start jolts occur.

### 3 — PWM Dead-band
**Problem:** Below a certain voltage the motor cannot overcome gearbox static
friction. Applying a tiny PWM just heats the windings with no shaft movement.

**How it works:** `apply_deadband()` forces any PWM whose magnitude is below
`PWM_DEADBAND` (15) to zero before the signal reaches the BTS7960.

**Tuning `PWM_DEADBAND`:** Command a very slow target speed. Lower `PWM_DEADBAND`
until the wheel just barely starts turning, then set it 2–3 counts above that
threshold. If wheels twitch at zero command raise to 20; if the robot won't start
moving at slow Nav2 commands lower to 10.

### 4 — Encoder Sanity Check
**Problem:** Motor PWM switching injects electrical noise into encoder wires,
which can register as fake pulses. A single noisy tick can make the firmware think
a wheel spun at 50 m/s, causing the PID to respond with a violent full-PWM jerk.

**How it works:** After computing `actual_v_left` and `actual_v_right` each tick,
the firmware compares them against `MAX_PHYSICAL_SPEED` (1.4 m/s — above the
motor's rated 1.2 m/s maximum). If either wheel reports an impossible speed, the
entire tick is discarded, the encoder baseline is reset, and the function returns
early. Motors are not driven with bad data.

This fires silently — there is no counter or log. If the robot jerks randomly,
check encoder cable routing (away from motor wires) and add ferrite beads.

### Protection summary
| Risk | Protection | Config constant |
|---|---|---|
| Blocked / stall burn | Stall detection — cuts power after timeout | `STALL_PWM_THRESHOLD`, `STALL_SPEED_THRESHOLD`, `STALL_TIME_MS` |
| Inrush current / gearbox jerk | Slew rate limiter on target velocity | `MAX_ACCEL_PER_TICK` |
| Low-PWM winding heat | PWM dead-band filter | `PWM_DEADBAND` |
| Runaway on comms loss | CMD timeout — stops if no cmd_vel for 500 ms | `CMD_TIMEOUT_MS` |
| Integral windup surge | Anti-windup clamp in PID | `PID_MIN_OUTPUT`, `PID_MAX_OUTPUT` |
| Derivative spike on start/retune | First-call guard in PID | — (flag in firmware) |
| Encoder noise jerk | Encoder sanity check — skips impossible-speed ticks | `MAX_PHYSICAL_SPEED` |

## PID Tuning
Send new gains at runtime via rqt Topic Publisher → `/pid_gains` (std_msgs/String):
Gains take effect immediately. Integrals are reset and the first-call guard is
re-armed on every gain change so there is no derivative spike after retuning.

Default gains in `config.h`: `KP=80, KI=5, KD=1` for both wheels.

### Tuning order
1. **Kp only** (Ki=0, Kd=0). Command 0.2 m/s. Raise Kp until the robot reaches
   ~90% of target speed. Starting point: 80. Back off if speed oscillates.
2. **Ki** after Kp is stable. Start at 1. Watch `/pid_debug` `data[2]` converge
   to zero in PlotJuggler. Halve Ki if wheels start oscillating.
3. **Kd** last, only if overshoot remains after Ki is set. Start at 0.5.
   72 000 CPR gives a smooth velocity signal so Kd up to ~2.0 is tolerable.
   Keep it small — gearbox backlash still injects some noise.

Write final values back into `config.h` (`KP_LEFT`, `KI_LEFT`, etc.) so they
survive a power cycle.