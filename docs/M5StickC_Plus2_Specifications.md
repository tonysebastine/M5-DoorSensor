# M5StickC Plus 2 Specifications

## 1. Core System
- **MCU**: ESP32-PICO-V3-02
  - **Core**: Dual-Core Xtensa® 32-bit LX6
  - **Frequency**: 240MHz
  - **Flash**: 8MB SPI Flash
  - **SRAM**: 520KB
  - **PSRAM**: 2MB (Optional, model dependent)
  - **Wi-Fi**: 2.4GHz 802.11 b/g/n
  - **Bluetooth**: BLE 4.2 + Classic

## 2. Onboard Peripherals
| Component | Detection / Driver | GPIO / Bus |
| :--- | :--- | :--- |
| **Display** | 1.14" TFT LCD (135x240) | ST7789v2 (SPI) |
| **IMU** | 6-Axis Accelerometer/Gyro | MPU6886 (I2C: 0x68) |
| **Microphone** | Digital MEMS | SPM1423 (I2S) |
| **RTC** | Real-Time Clock | BM8563 (I2C: 0x51) |
| **Button A** | Main / Select | GPIO 37 |
| **Button B** | Side / Option | GPIO 39 |
| **Button C** | Power / Reset | PMIC (AXP2101/192) |
| **Red LED** | Status Indicator | GPIO 19 |
| **IR Transmitter** | Infrared LED | GPIO 19 |
| **Buzzer** | Passive Piezo | GPIO 2 |
| **Power Hold** | Keep System Alive | GPIO 4 |

## 3. Power Management
- **Battery**: 200mAh @ 3.7V LiPo
- **PMIC**: AXP2101 or AXP192 (Handles charging & rails)
- **Voltage Monitoring**: 
  - Internal ADCs often used or PMIC reading.
  - VBUS (USB) and VBAT monitoring via PMIC.

## 4. Expansion Ports
### Grove Port (HY2.0-4P)
| Pin | Function | Description |
| :--- | :--- | :--- |
| **G32** | SDA / Signal | General Purpose I/O, I2C SDA |
| **G33** | SCL / Signal | General Purpose I/O, I2C SCL |
| **5V** | VCC | 5V Output (USB/Battery Boost) |
| **GND** | Ground | Common Ground |

### Hat Header (Top)
- **G0, G25/G36**: General GPIOs available for HATs.
- **3.3V, 5V, GND, BAT**: Power rails.

## 5. Firmware Configuration (Current Project)
The **M5-DoorSensor** firmware utilizes specific pins:
- **Door State Detection**: MPU6886 Accelerometer (I2C) - Detects Gravity/Orientation.
- **Display**: ST7789 via SPI (Landscape, 240x135).
- **WiFi**: ESP32 Station Mode (Fast Connect, Static IP).
- **Battery**: Internal ADC Check (G38/ADC1_CH2 or similar).
- **Sleep Mode**: Deep Sleep with Timer & GPIO Wakeup.
- **Boot**: GPIO 4 Hold to keep power on during boot.
