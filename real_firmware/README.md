# real_firmware — ESP32 Arduino Firmware

## Arduino IDE Setup
| Item | Version |
|---|---|
| Arduino IDE | 2.x |
| ESP32 board core | **2.0.2** (critical — newer versions break micro-ROS timers) |
| micro_ros_arduino | jazzy release — install as .ZIP from GitHub |
| MPU6050 library | by Electronic Cats — via Library Manager |

Install ESP32 core 2.0.2:
Boards Manager URL: `https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json`
Search "esp32" → install version **2.0.2** specifically.

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

### Encoders (MY-37)
| Signal | ESP32 GPIO |
|---|---|
| Left encoder  | 18 |
| Right encoder | 19 |
| VCC | 3.3V |
| GND | GND |

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

# reate rules file
sudo nano /etc/udev/rules.d/99-robot.rules
```

 (replace idVendor/idProduct with actual values):