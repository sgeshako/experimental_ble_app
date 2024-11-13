# Nordic BLE HID Keyboard Project
Bluetooth Low Energy (BLE) HID Keyboard: Communicates Left/Right arrow key events over BLE based on Gyroscope/Accelerometer data.

- Uses combination of Nordic Blinky and HID Keyboard SDK examples
- SPI communication with LSM6DS33 IMU
- Bare-metal application with super loop
- nRF52832 ARM SoC / Nordic SDK 12.2.0 / SoftDevice S132

## Project Structure
```plaintext
├── main.c                  # Main application
├── filters.c               # Digital filter implementations
├── /spi/                   # SPI communication to sensor
├── /pwm/                   # PWM dimming of LED indicates when advertising is active
├── /services/              # Custom BLE services
├── /hid/                   # BLE HID service and nRF Peer Manager functionality
└── /pca10040/s132/
    ├── armgcc/Makefile     # Project Makefile
    └── config/sdk_config.h # Nordic SDK configuration file
```
