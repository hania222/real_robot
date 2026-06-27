# Carriage Subsystem — Wiring Reference

## Active-LOW wiring (negative pins → ESP32 GPIOs, positive pins → 3.3V)
ESP32 is 3.3V logic — add a 1kΩ resistor in series on each signal line (PUL, DIR, ENA).

| From | To | Note |
|---|---|---|
| ESP32 GPIO 18 | 1kΩ resistor → TB6600 Rot PUL− | Rotation step pulse |
| ESP32 GPIO 19 | 1kΩ resistor → TB6600 Rot DIR− | Rotation direction |
| ESP32 GPIO 21 | 1kΩ resistor → TB6600 Rot ENA− | Rotation enable (LOW = enabled) |
| ESP32 GPIO 22 | 1kΩ resistor → TB6600 Ext1 PUL− | Extension step pulse (both drivers) |
| ESP32 GPIO 22 | 1kΩ resistor → TB6600 Ext2 PUL− | Same GPIO wired to both |
| ESP32 GPIO 23 | 1kΩ resistor → TB6600 Ext1 DIR− | Extension direction (both drivers) |
| ESP32 GPIO 23 | 1kΩ resistor → TB6600 Ext2 DIR− | Same GPIO wired to both |
| ESP32 GPIO 25 | 1kΩ resistor → TB6600 Ext1 ENA− | Extension driver 1 enable |
| ESP32 GPIO 26 | 1kΩ resistor → TB6600 Ext2 ENA− | Extension driver 2 enable |
| ESP32 3.3V | TB6600 Rot PUL+ |  |
| ESP32 3.3V | TB6600 Rot DIR+ | |
| ESP32 3.3V | TB6600 Rot ENA+ | |
| ESP32 3.3V | TB6600 Ext1 PUL+ | |
| ESP32 3.3V | TB6600 Ext1 DIR+ | |
| ESP32 3.3V | TB6600 Ext1 ENA+ | |
| ESP32 3.3V | TB6600 Ext2 PUL+ | |
| ESP32 3.3V | TB6600 Ext2 DIR+ | |
| ESP32 3.3V | TB6600 Ext2 ENA+ | |
| ESP32 GND | TB6600 Rot GND (signal) |  |
| ESP32 GND | TB6600 Ext1 GND (signal) | |
| ESP32 GND | TB6600 Ext2 GND (signal) | |
| ESP32 GPIO 32 | Arm home switch leg A | LOW = triggered |
| ESP32 GPIO 13 | Arm end switch leg A | |
| ESP32 GPIO 33 | Rotation home switch leg A | LOW = triggered |
| ESP32 GND | All switch leg B | |
| ESP32 GPIO 27 | Servo left signal | SG90 signal wire |
| ESP32 GPIO 14 | Servo right signal | SG90 signal wire |
| External 5V rail | Both SG90 red wire (VCC) | Do NOT power servos from ESP32 5V pin |
| External 5V GND | Both SG90 brown wire (GND) | |
| External 5V GND | ESP32 GND | Common ground between servo rail and ESP32 |
| 24V PSU positive | TB6600 Rot VCC | Motor power |
| 24V PSU positive | TB6600 Ext1 VCC | |
| 24V PSU positive | TB6600 Ext2 VCC | |
| 24V PSU negative | TB6600 Rot GND (power) | |
| 24V PSU negative | TB6600 Ext1 GND (power) | |
| 24V PSU negative | TB6600 Ext2 GND (power) | |
| 24V PSU negative | ESP32 GND | Star ground — tie all GNDs together |
| TB6600 Rot A+ | Rotation NEMA23 coil A+ | |
| TB6600 Rot A− | Rotation NEMA23 coil A− | |
| TB6600 Rot B+ | Rotation NEMA23 coil B+ | |
| TB6600 Rot B− | Rotation NEMA23 coil B− | |
| TB6600 Ext1 A+ | Extension NEMA17 motor 1 coil A+ | |
| TB6600 Ext1 A− | Extension NEMA17 motor 1 coil A− | |
| TB6600 Ext1 B+ | Extension NEMA17 motor 1 coil B+ | |
| TB6600 Ext1 B− | Extension NEMA17 motor 1 coil B− | |
| TB6600 Ext2 A+ | Extension NEMA17 motor 2 coil A+ | |
| TB6600 Ext2 A− | Extension NEMA17 motor 2 coil A− | |
| TB6600 Ext2 B+ | Extension NEMA17 motor 2 coil B+ | |
| TB6600 Ext2 B− | Extension NEMA17 motor 2 coil B− | |

Firmware: `enable_driver()` writes LOW, `disable_driver()` writes HIGH.
`DRIVER_ENABLE_LEVEL LOW`, `DRIVER_DISABLE_LEVEL HIGH` in config.

---

## Active-HIGH wiring (positive pins → ESP32 GPIOs, negative pins → GND)
Simpler for 3.3V ESP32 — no resistors needed because GND reference is solid.
The optocoupler sees the full 3.3V swing directly

| From | To | Note |
|---|---|---|
| ESP32 GPIO 18 | TB6600 Rot PUL+ | Rotation step pulse |
| ESP32 GPIO 19 | TB6600 Rot DIR+ | Rotation direction |
| ESP32 GPIO 21 | TB6600 Rot ENA+ | Rotation enable (HIGH = enabled) |
| ESP32 GPIO 22 | TB6600 Ext1 PUL+ | Extension step pulse (both drivers) |
| ESP32 GPIO 22 | TB6600 Ext2 PUL+ | Same GPIO wired to both |
| ESP32 GPIO 23 | TB6600 Ext1 DIR+ | Extension direction (both drivers) |
| ESP32 GPIO 23 | TB6600 Ext2 DIR+ | Same GPIO wired to both |
| ESP32 GPIO 25 | TB6600 Ext1 ENA+ | Extension driver 1 enable |
| ESP32 GPIO 26 | TB6600 Ext2 ENA+ | Extension driver 2 enable |
| ESP32 GND | TB6600 Rot PUL− | -ve side of optocoupler tied to GND |
| ESP32 GND | TB6600 Rot DIR− | |
| ESP32 GND | TB6600 Rot ENA− | |
| ESP32 GND | TB6600 Ext1 PUL− | |
| ESP32 GND | TB6600 Ext1 DIR− | |
| ESP32 GND | TB6600 Ext1 ENA− | |
| ESP32 GND | TB6600 Ext2 PUL− | |
| ESP32 GND | TB6600 Ext2 DIR− | |
| ESP32 GND | TB6600 Ext2 ENA− | |
| ESP32 GND | TB6600 Rot GND (signal) | Common ground |
| ESP32 GND | TB6600 Ext1 GND (signal) | |
| ESP32 GND | TB6600 Ext2 GND (signal) | |
| ESP32 GPIO 32 | Arm home switch leg A | INPUT_PULLUP, LOW = triggered |
| ESP32 GPIO 13 | Arm end switch leg A | INPUT_PULLUP, LOW = triggered  |
| ESP32 GPIO 33 | Rotation home switch leg A | INPUT_PULLUP, LOW = triggered |
| ESP32 GND | All switch leg B | |
| ESP32 GPIO 27 | Servo left signal | SG90 signal wire |
| ESP32 GPIO 14 | Servo right signal | SG90 signal wire |
| External 5V rail | Both SG90 red wire (VCC) |  |
| External 5V GND | Both SG90 brown wire (GND) | |
| External 5V GND | ESP32 GND | Common ground between servo rail and ESP32 |
| 24V PSU positive | TB6600 Rot VCC | Motor power |
| 24V PSU positive | TB6600 Ext1 VCC | |
| 24V PSU positive | TB6600 Ext2 VCC | |
| 24V PSU negative | TB6600 Rot GND (power) | |
| 24V PSU negative | TB6600 Ext1 GND (power) | |
| 24V PSU negative | TB6600 Ext2 GND (power) | |
| 24V PSU negative | ESP32 GND | all GNDs together |
| TB6600 Rot A+ | Rotation NEMA23 coil A+ | |
| TB6600 Rot A− | Rotation NEMA23 coil A− | |
| TB6600 Rot B+ | Rotation NEMA23 coil B+ | |
| TB6600 Rot B− | Rotation NEMA23 coil B− | |
| TB6600 Ext1 A+ | Extension NEMA17 motor 1 coil A+ | |
| TB6600 Ext1 A− | Extension NEMA17 motor 1 coil A− | |
| TB6600 Ext1 B+ | Extension NEMA17 motor 1 coil B+ | |
| TB6600 Ext1 B− | Extension NEMA17 motor 1 coil B− | |
| TB6600 Ext2 A+ | Extension NEMA17 motor 2 coil A+ | |
| TB6600 Ext2 A− | Extension NEMA17 motor 2 coil A− | |
| TB6600 Ext2 B+ | Extension NEMA17 motor 2 coil B+ | |
| TB6600 Ext2 B− | Extension NEMA17 motor 2 coil B− | |

Firmware: `enable_driver()` writes HIGH, `disable_driver()` writes LOW.
`DRIVER_ENABLE_LEVEL HIGH`, `DRIVER_DISABLE_LEVEL LOW` in config.

---

# Lift Subsystem — Wiring Reference
| From | To |
|---|---|
| Arduino D4 | TB6600 Driver 1 PUL− |
| Arduino D4 | TB6600 Driver 2 PUL− |
| Arduino D5 | TB6600 Driver 1 DIR− |
| Arduino D5 | TB6600 Driver 2 DIR− |
| Arduino D6 | TB6600 Driver 1 ENA− |
| Arduino D7 | TB6600 Driver 2 ENA− |
| Arduino D8 | Bottom limit switch leg A |
| Arduino D9 | Top limit switch leg A |
| Arduino 5V | TB6600 Driver 1 PUL+ |
| Arduino 5V | TB6600 Driver 1 DIR+ |
| Arduino 5V | TB6600 Driver 1 ENA+ |
| Arduino 5V | TB6600 Driver 2 PUL+ |
| Arduino 5V | TB6600 Driver 2 DIR+ |
| Arduino 5V | TB6600 Driver 2 ENA+ |
| Arduino GND | TB6600 Driver 1 GND (signal) |
| Arduino GND | TB6600 Driver 2 GND (signal) |
| Arduino GND | Bottom limit switch leg B |
| Arduino GND | Top limit switch leg B |
| Arduino GND | 24V PSU negative |
| 24V PSU positive | TB6600 Driver 1 VCC |
| 24V PSU positive | TB6600 Driver 2 VCC |
| 24V PSU negative | TB6600 Driver 1 GND (power) |
| 24V PSU negative | TB6600 Driver 2 GND (power) |
| TB6600 Driver 1 A+ | Motor 1 coil A+ |
| TB6600 Driver 1 A− | Motor 1 coil A− |
| TB6600 Driver 1 B+ | Motor 1 coil B+ |
| TB6600 Driver 1 B− | Motor 1 coil B− |
| TB6600 Driver 2 A+ | Motor 2 coil A+ |
| TB6600 Driver 2 A− | Motor 2 coil A− |
| TB6600 Driver 2 B+ | Motor 2 coil B+ |
| TB6600 Driver 2 B− | Motor 2 coil B− |