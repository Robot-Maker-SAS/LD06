## LD06 Arduino Library

**Author:** Jonathan QUILLES (Mike118)  
**Sensor:** [LD06 LiDAR](https://www.robot-maker.com/shop/capteurs/468-lidar-ld06-468.html)  
**Version:** 2025

---

## Description

This library provides a robust and easy-to-use interface for the **LD06 LiDAR**, a compact 2D laser distance scanner capable of a full **360° horizontal scan**.  
It handles packet decoding, angle reconstruction, CRC checking, and full 360° scan detection.

Each data packet from the LD06 contains **12 measurements**, including distance, intensity, and angular information.  
The library reconstructs all intermediate angles and provides access to both **polar** (angle, distance, intensity) and **Cartesian** (x, y) coordinates.

---

## Features

- Decode LD06 UART packets (12 points per frame)

- Automatic 360° scan detection

- Per-point data access: angle, distance, intensity

- Optional Cartesian coordinate computation *(x, y)*

- Built-in CRC validation and packet resynchronization

- Support for **Teleplot** real-time visualization

- Optional filtering by distance or intensity

- Optional “upside-down” mounting compensation

- Lightweight and memory-safe (no `String` objects, static buffers only)

---

## Hardware Specifications

| Parameter           | Typical Value   |
| ------------------- | --------------- |
| Interface           | UART (3.3V TTL) |
| Baud rate           | 230400 bps      |
| Points per packet   | 12              |
| Rotation speed      | 10–15 Hz        |
| Angular resolution  | 0.5°–1°         |
| Maximum range       | ~12 m           |
| Packet size         | 47–48 bytes     |
| Header byte         | 0x54            |
| Version/length byte | 0x2C            |

---

## Example: Basic Usage

```cpp
#include <LD06.h> 

LD06 lidar(Serial1); 
void setup() { 
    Serial.begin(115200); // Debug output 
    lidar.init(); // Initialize UART at 230400 baud 
    lidar.enableCRC(); // Enable checksum validation 
    lidar.enableFullScan(); // Detect full 360° scans 
} 

void loop() { 
    if (lidar.readScan()) { // Returns true when a full 360° scan is ready 
        lidar.printScanCSV(Serial); 
    } 
}
```

### Example CSV Output

`N,Angle(°),Distance(mm),Intensity,x(mm),y(mm) 0,0.50,623,12,622.8,5.4 1,1.50,624,11,623.4,16.2 ...`

---

## Real-Time Visualization (Teleplot)

You can visualize scan data in real time using [Teleplot](https://teleplot.fr/).  
Simply print the scan in the expected format:

```cpp
#include <LD06.h>

LD06 lidar(Serial1);

void setup() { 
    Serial.begin(115200); 
    lidar.init();
    lidar.enableFullScan(); 
}

void loop() { 
    if (lidar.readScan()) {
        lidar.printScanTeleplot(Serial); // Sends data in Teleplot XY format
    } 
}`
```

On Teleplot, select **XY mode** to display the live LiDAR point cloud.

---

## API Reference

| Function                                   | Description                                                         |
| ------------------------------------------ | ------------------------------------------------------------------- |
| `init()`                                   | Initializes the serial interface and powers the LiDAR               |
| `readScan()`                               | Reads data; returns `true` when a new scan (or packet) is available |
| `enableCRC()` / `disableCRC()`             | Enable or disable checksum validation                               |
| `enableFullScan()` / `disableFullScan()`   | Choose between full 360° scans or per-packet updates                |
| `enableFiltering()` / `disableFiltering()` | Enable or disable distance/intensity filtering                      |
| `printScanCSV(Stream &)`                   | Output the latest scan as CSV data                                  |
| `printScanTeleplot(Stream &)`              | Output in Teleplot-compatible format                                |
| `setUpsideDown(bool)`                      | Inverts rotation direction for upside-down mounting                 |
| `setDistanceRange(min, max)`               | Defines the valid distance range (mm)                               |
| `setOffsetPosition(x, y, θ)`               | Defines mechanical offset and rotation of the LiDAR                 |

---

## Usage Notes

- The LD06 should be powered at **5V**, but uses **3.3V TTL** UART logic.

- A **hardware serial port** is required (e.g. `Serial1`, `Serial2`);  
  *SoftwareSerial is too slow for 230400 baud.*

- Call `readScan()` regularly in your main loop — it is **non-blocking**.

- If the sensor is mounted upside-down, call `setUpsideDown(true)` to correct the angle direction.

---

## Compatibility

| Platform                              | Status                      |
| ------------------------------------- | --------------------------- |
| ESP32 / STM32 / Teensy                | Supported                   |
| Arduino Uno / Nano / Arduino Mega2560 | Limited (low RAM and speed) |
| Raspberry Pi Pico                     | To be confirmed             |

---

## License

**MIT License**  
Copyright (c) 2025  
**Author:** Jonathan QUILLES (Mike118)

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files to use, copy, modify, merge, publish, and distribute without restriction.