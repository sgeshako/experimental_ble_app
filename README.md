# Nordic BLE HID Keyboard Project
Bluetooth Low Energy (BLE) HID Keyboard: Communicates Left/Right arrow key events over BLE based on Gyroscope/Accelerometer data.

- Uses combination of SDK examples from Nordic - Blinky and HID Keyboard
- SPI communication with [LSM6DS33](https://www.pololu.com/file/0J1087/LSM6DS33.pdf) inertial module
- Bare-metal application with super loop
- nRF52832 ARM SoC / Nordic [SDK 12.2.0](https://docs.nordicsemi.com/bundle/sdk_nrf5_v12.2.0/page/index.html) / [SoftDevice S132](https://docs.nordicsemi.com/bundle/sds_s132/page/SDS/s1xx/s130.html)
  - Flash and debug with [IDAP-Link](https://www.i-syst.com/products/idap-link) J-Tag probe
      - Log to PC Terminal using built-in UART-to-USB bridge  
  - Custom devboard equipped with Button and LED from https://www.i-syst.com/products/blyst-nano
  - IDE: Eclipse Embedded CDT for Arm development

## Project Structure
```plaintext
├── main.c                  # Main application
├── filters.c               # Digital filter implementations
├── /spi/                   # SPI communication to sensor
├── /pwm/                   # PWM dimming of LED indicates when advertising is active
├── /services/              # Custom BLE services
├── /hid/                   # BLE HID service and nRF Peer Manager functionality
└── /pca10040/s132/n
    ├── armgcc/Makefile     # Project Makefile
    └── config/sdk_config.h # Nordic SDK configuration file
```

## Use-case example
1. Compile and program application.
2. Advertising starts:
   1. Fast mode for 30 seconds (LED dims<->brightens quickly)
   2. Slow mode for 180 seconds (LED dims<->brightens slowly)
   3. Idle mode. Application goes to sleep.
3. SPI Driver is initialized and SPI write command configures inertial sensor operating mode. 
4. Connect to HID Keyboard application from PC/Phone.
5. Read/write GATT Services and characterstics on application using [Bluetooth LE Lab](https://apps.microsoft.com/detail/9n6jd37gwzc8?hl=en-US&gl=US) on Windows or [nRF Connect](https://www.nordicsemi.com/Products/Development-tools/nRF-Connect-for-mobile) on Mobile
   - Write 0xF0 to custom IMS service starts 5ms Timer for sampling intertial sensor over SPI and writing samples to buffer.
   - Write 0x00 to custom IMS service stops Timer.
6. Every 100ms (when buffer gets full) the HID application sends Left/Right key arrow commands based on sensor tilted above or below +/-25 degrees.

#### Disconnect HID application by removing it from Bluetooth devices list on Windows:
1. Application starts advertising:
   1. Fast mode with whitelist (LED goes from dim to bright quickly) for 30 seconds.
   2. Slow mode with whitelist (LED goes from bright to dim slowly) for 180 seconds.
   3. Idle mode. Application goes to sleep.
   
